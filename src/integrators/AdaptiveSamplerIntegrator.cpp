// core/integrator.cpp*
#include "integrator.h"
#include "scene.h"
#include "interaction.h"
#include "sampling.h"
#include "parallel.h"
#include "film.h"
#include "sampler.h"
#include "integrator.h"
#include "progressreporter.h"
#include "camera.h"
#include "stats.h"
#include "AdaptiveSamplerIntegrator.h"

namespace pbrt 
{

    STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

    // SamplerIntegrator Method Definitions
    void AdaptiveSamplerIntegrator::Render(const Scene &scene) 
    {
        // Initialize values
        Preprocess(scene, *sampler);
        sampleBounds = camera->film->GetSampleBounds();
        sampleExtent = sampleBounds.Diagonal();
        nTiles = Point2i((sampleExtent.x + tileSize - 1) / tileSize,
                         (sampleExtent.y + tileSize - 1) / tileSize);
        

        //TODO::
        //Invoke CreateSampleMap
        //Adjust the render / processing methods to take in the sample map and render pixels as specified

        ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");

        ParallelFor2D([&](Point2i tile) // Render section of image corresponding to _tile_
        {
            AdaptiveIteration(tile);
            reporter.Update();
        }, nTiles);

        reporter.Done();

        LOG(INFO) << "Rendering finished";

        // Save final image after rendering
        camera->film->WriteImage();
    }

    Bounds2i AdaptiveSamplerIntegrator::BoundsForTile(const Point2i tile)
    {
        // Compute sample bounds for tile
        int x0 = sampleBounds.pMin.x + tile.x * tileSize;
        int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
        int y0 = sampleBounds.pMin.y + tile.y * tileSize;
        int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
        Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));

        return tileBounds;
    }

    void AdaptiveSamplerIntegrator::RenderTile(const Scene &scene, const Point2i tile)
    {
        MemoryArena arena; // Allocate _MemoryArena_ for tile
        Bounds2i tileBounds = BoundsForTile(tile);
        int seed = tile.y * nTiles.x + tile.x; // Sampler seed is based on index of the tile
        std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed); // Get sampler instance for tile

        LOG(INFO) << "Starting image tile " << tileBounds;

        // Get _FilmTile_ for tile
        std::unique_ptr<FilmTile> filmTile = camera->film->GetFilmTile(tileBounds);

        // Loop over pixels in tile to render them
        for (Point2i pixel : tileBounds)
        {
            {
                ProfilePhase pp(Prof::StartPixel);
                tileSampler->StartPixel(pixel);
            }

            // Do this check after the StartPixel() call; this keeps
            // the usage of RNG values from (most) Samplers that use
            // RNGs consistent, which improves reproducability /
            // debugging.
            // Checks if the pixel is inside bounds...
            if (!InsideExclusive(pixel, pixelBounds)) continue;

            do
            {
                // Initialize _CameraSample_ for current sample
                CameraSample cameraSample = tileSampler->GetCameraSample(pixel);

                // Generate camera ray for current sample
                RayDifferential ray;
                Float rayWeight = camera->GenerateRayDifferential(cameraSample, &ray);
                ray.ScaleDifferentials(1 / std::sqrt((Float)tileSampler->samplesPerPixel));
                ++nCameraRays;

                // Evaluate radiance along camera ray
                Spectrum radiance(0.f);
                if (rayWeight > 0) radiance = Li(ray, scene, *tileSampler, arena);

                CheckRadiance(radiance, pixel, tileSampler); // Ensure radiance is within limits and valid

                VLOG(1) << "Camera sample: " << cameraSample << " -> ray: " << ray << " -> radiance = " << radiance;

                // Add camera ray's contribution to image
                filmTile->AddSample(cameraSample.pFilm, radiance, rayWeight);

                // Free _MemoryArena_ memory from computing image sample
                // value
                arena.Reset();
            } while (tileSampler->StartNextSample());
        }

        LOG(INFO) << "Finished image tile " << tileBounds;

        // Merge image tile into _Film_
        camera->film->MergeFilmTile(std::move(filmTile));
    }

    void AdaptiveSamplerIntegrator::RenderTile(const Point2i tile)
    {
        
    }

    void AdaptiveSamplerIntegrator::CheckRadiance(Spectrum &radiance, const Point2i pixel, const std::unique_ptr<Sampler> &sampler)
    {
        // Issue warning if unexpected radiance value returned
        if (radiance.HasNaNs())
        {
            LOG(ERROR) << StringPrintf(
                "Not-a-number radiance value returned "
                "for pixel (%d, %d), sample %d. Setting to black.",
                pixel.x, pixel.y,
                (int)sampler->CurrentSampleNumber());
            radiance = Spectrum(0.f);
        }
        else if (radiance.y() < -1e-5)
        {
            LOG(ERROR) << StringPrintf(
                "Negative luminance value, %f, returned "
                "for pixel (%d, %d), sample %d. Setting to black.",
                radiance.y(), pixel.x, pixel.y,
                (int)sampler->CurrentSampleNumber());
            radiance = Spectrum(0.f);
        }
        else if (std::isinf(radiance.y()))
        {
            LOG(ERROR) << StringPrintf(
                "Infinite luminance value returned "
                "for pixel (%d, %d), sample %d. Setting to black.",
                pixel.x, pixel.y,
                (int)sampler->CurrentSampleNumber());
            radiance = Spectrum(0.f);
        }
    }

Spectrum AdaptiveSamplerIntegrator::SpecularReflect(
    const RayDifferential &ray, const SurfaceInteraction &isect,
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const {
    // Compute specular reflection direction _wi_ and BSDF value
    Vector3f wo = isect.wo, wi;
    Float pdf;
    BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
    Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

    // Return contribution of specular reflection
    const Normal3f &ns = isect.shading.n;
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular reflection
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = isect.p + isect.dpdx;
            rd.ryOrigin = isect.p + isect.dpdy;
            // Compute differential reflected directions
            Normal3f dndx = isect.shading.dndu * isect.dudx +
                isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                isect.shading.dndv * isect.dvdy;
            Vector3f dwodx = -ray.rxDirection - wo,
                dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);
            rd.rxDirection =
                wi - dwodx + 2.f * Vector3f(Dot(wo, ns) * dndx + dDNdx * ns);
            rd.ryDirection =
                wi - dwody + 2.f * Vector3f(Dot(wo, ns) * dndy + dDNdy * ns);
        }
        return f * Li(rd, scene, sampler, arena, depth + 1) * AbsDot(wi, ns) /
            pdf;
    }
    else
        return Spectrum(0.f);
}

Spectrum AdaptiveSamplerIntegrator::SpecularTransmit(
    const RayDifferential &ray, const SurfaceInteraction &isect,
    const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const {
    Vector3f wo = isect.wo, wi;
    Float pdf;
    const Point3f &p = isect.p;
    const Normal3f &ns = isect.shading.n;
    const BSDF &bsdf = *isect.bsdf;
    Spectrum f = bsdf.Sample_f(wo, &wi, sampler.Get2D(), &pdf,
        BxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR));
    Spectrum L = Spectrum(0.f);
    if (pdf > 0.f && !f.IsBlack() && AbsDot(wi, ns) != 0.f) {
        // Compute ray differential _rd_ for specular transmission
        RayDifferential rd = isect.SpawnRay(wi);
        if (ray.hasDifferentials) {
            rd.hasDifferentials = true;
            rd.rxOrigin = p + isect.dpdx;
            rd.ryOrigin = p + isect.dpdy;

            Float eta = bsdf.eta;
            Vector3f w = -wo;
            if (Dot(wo, ns) < 0) eta = 1.f / eta;

            Normal3f dndx = isect.shading.dndu * isect.dudx +
                isect.shading.dndv * isect.dvdx;
            Normal3f dndy = isect.shading.dndu * isect.dudy +
                isect.shading.dndv * isect.dvdy;

            Vector3f dwodx = -ray.rxDirection - wo,
                dwody = -ray.ryDirection - wo;
            Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
            Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);

            Float mu = eta * Dot(w, ns) - Dot(wi, ns);
            Float dmudx =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdx;
            Float dmudy =
                (eta - (eta * eta * Dot(w, ns)) / Dot(wi, ns)) * dDNdy;

            rd.rxDirection =
                wi + eta * dwodx - Vector3f(mu * dndx + dmudx * ns);
            rd.ryDirection =
                wi + eta * dwody - Vector3f(mu * dndy + dmudy * ns);
        }
        L = f * Li(rd, scene, sampler, arena, depth + 1) * AbsDot(wi, ns) / pdf;
    }
    return L;
}



}  // namespace pbrt