
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

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

namespace pbrt {

STAT_COUNTER("Integrator/Camera rays traced", nCameraRays);

// Integrator Method Definitions
Integrator::~Integrator() {}

// Integrator Utility Functions
Spectrum UniformSampleAllLights(const Interaction &it, const Scene &scene,
                                MemoryArena &arena, Sampler &sampler,
                                const std::vector<int> &nLightSamples,
                                bool handleMedia) {
    ProfilePhase p(Prof::DirectLighting);
    Spectrum L(0.f);
    for (size_t j = 0; j < scene.lights.size(); ++j) {
        // Accumulate contribution of _j_th light to _L_
        const std::shared_ptr<Light> &light = scene.lights[j];
        int nSamples = nLightSamples[j];
        const Point2f *uLightArray = sampler.Get2DArray(nSamples);
        const Point2f *uScatteringArray = sampler.Get2DArray(nSamples);
        if (!uLightArray || !uScatteringArray) {
            // Use a single sample for illumination from _light_
            Point2f uLight = sampler.Get2D();
            Point2f uScattering = sampler.Get2D();
            L += EstimateDirect(it, uScattering, *light, uLight, scene, sampler,
                                arena, handleMedia);
        } else {
            // Estimate direct lighting using sample arrays
            Spectrum Ld(0.f);
            for (int k = 0; k < nSamples; ++k)
                Ld += EstimateDirect(it, uScatteringArray[k], *light,
                                     uLightArray[k], scene, sampler, arena,
                                     handleMedia);
            L += Ld / nSamples;
        }
    }
    return L;
}

Spectrum UniformSampleOneLight(const Interaction &it, const Scene &scene,
                               MemoryArena &arena, Sampler &sampler,
                               bool handleMedia, const Distribution1D *lightDistrib) {
    ProfilePhase p(Prof::DirectLighting);
    // Randomly choose a single light to sample, _light_
    int nLights = int(scene.lights.size());
    if (nLights == 0) return Spectrum(0.f);
    int lightNum;
    Float lightPdf;
    if (lightDistrib) {
        lightNum = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightPdf);
        if (lightPdf == 0) return Spectrum(0.f);
    } else {
        lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
        lightPdf = Float(1) / nLights;
    }
    const std::shared_ptr<Light> &light = scene.lights[lightNum];
    Point2f uLight = sampler.Get2D();
    Point2f uScattering = sampler.Get2D();
    return EstimateDirect(it, uScattering, *light, uLight,
                          scene, sampler, arena, handleMedia) / lightPdf;
}

Spectrum EstimateDirect(const Interaction &it, const Point2f &uScattering,
                        const Light &light, const Point2f &uLight,
                        const Scene &scene, Sampler &sampler,
                        MemoryArena &arena, bool handleMedia, bool specular) {
    BxDFType bsdfFlags =
        specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
    Spectrum Ld(0.f);
    // Sample light source with multiple importance sampling
    Vector3f wi;
    Float lightPdf = 0, scatteringPdf = 0;
    VisibilityTester visibility;
    Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
    VLOG(2) << "EstimateDirect uLight:" << uLight << " -> Li: " << Li << ", wi: "
            << wi << ", pdf: " << lightPdf;
    if (lightPdf > 0 && !Li.IsBlack()) {
        // Compute BSDF or phase function's value for light sample
        Spectrum f;
        if (it.IsSurfaceInteraction()) {
            // Evaluate BSDF for light sampling strategy
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->f(isect.wo, wi, bsdfFlags) *
                AbsDot(wi, isect.shading.n);
            scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);
            VLOG(2) << "  surf f*dot :" << f << ", scatteringPdf: " << scatteringPdf;
        } else {
            // Evaluate phase function for light sampling strategy
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->p(mi.wo, wi);
            f = Spectrum(p);
            scatteringPdf = p;
            VLOG(2) << "  medium p: " << p;
        }
        if (!f.IsBlack()) {
            // Compute effect of visibility for light source sample
            if (handleMedia) {
                Li *= visibility.Tr(scene, sampler);
                VLOG(2) << "  after Tr, Li: " << Li;
            } else {
              if (!visibility.Unoccluded(scene)) {
                VLOG(2) << "  shadow ray blocked";
                Li = Spectrum(0.f);
              } else
                VLOG(2) << "  shadow ray unoccluded";
            }

            // Add light's contribution to reflected radiance
            if (!Li.IsBlack()) {
                if (IsDeltaLight(light.flags))
                    Ld += f * Li / lightPdf;
                else {
                    Float weight =
                        PowerHeuristic(1, lightPdf, 1, scatteringPdf);
                    Ld += f * Li * weight / lightPdf;
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if (!IsDeltaLight(light.flags)) {
        Spectrum f;
        bool sampledSpecular = false;
        if (it.IsSurfaceInteraction()) {
            // Sample scattered direction for surface interactions
            BxDFType sampledType;
            const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
            f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
                                     bsdfFlags, &sampledType);
            f *= AbsDot(wi, isect.shading.n);
            sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
        } else {
            // Sample scattered direction for medium interactions
            const MediumInteraction &mi = (const MediumInteraction &)it;
            Float p = mi.phase->Sample_p(mi.wo, &wi, uScattering);
            f = Spectrum(p);
            scatteringPdf = p;
        }
        VLOG(2) << "  BSDF / phase sampling f: " << f << ", scatteringPdf: " <<
            scatteringPdf;
        if (!f.IsBlack() && scatteringPdf > 0) {
            // Account for light contributions along sampled direction _wi_
            Float weight = 1;
            if (!sampledSpecular) {
                lightPdf = light.Pdf_Li(it, wi);
                if (lightPdf == 0) return Ld;
                weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
            }

            // Find intersection and compute transmittance
            SurfaceInteraction lightIsect;
            Ray ray = it.SpawnRay(wi);
            Spectrum Tr(1.f);
            bool foundSurfaceInteraction =
                handleMedia ? scene.IntersectTr(ray, sampler, &lightIsect, &Tr)
                            : scene.Intersect(ray, &lightIsect);

            // Add light contribution from material sampling
            Spectrum Li(0.f);
            if (foundSurfaceInteraction) {
                if (lightIsect.primitive->GetAreaLight() == &light)
                    Li = lightIsect.Le(-wi);
            } else
                Li = light.Le(ray);
            if (!Li.IsBlack()) Ld += f * Li * Tr * weight / scatteringPdf;
        }
    }
    return Ld;
}

std::unique_ptr<Distribution1D> ComputeLightPowerDistribution(
    const Scene &scene) {
    if (scene.lights.empty()) return nullptr;
    std::vector<Float> lightPower;
    for (const auto &light : scene.lights)
        lightPower.push_back(light->Power().y());
    return std::unique_ptr<Distribution1D>(
        new Distribution1D(&lightPower[0], lightPower.size()));
}




/************************************************************************/
/***************************SamplerIntegrator****************************/
/************************************************************************/

void SamplerIntegrator::Render(const Scene &scene)
{
    LOG(INFO) << "Render function executes";

    // Initialize values
    Preprocess(scene, *sampler);
    sampleBounds = camera->film->GetSampleBounds();
    sampleExtent = sampleBounds.Diagonal();
    nTiles = Point2i((sampleExtent.x + tileSize - 1) / tileSize,
                     (sampleExtent.y + tileSize - 1) / tileSize);

    sampler->InitializeSamplingPlan(camera->film);
    ProgressReporter reporter(nTiles.x * nTiles.y * sampler->samplingPlanner->plannedIterations + 1, "Rendering");
    do
    {
        LOG(INFO) << "Starting iteration " << sampler->samplingPlanner->currentIteration;

        sampler->UpdateSamplingPlan(camera->film); //Update the distribution of samples for this iteration

        ParallelFor2D([&](Point2i tile) // Render section of image corresponding to _tile_
        {
            //Render tiles for each buffer
            RenderTile(scene, tile);

            reporter.Update();
        }, nTiles);

    } 
    while (sampler->StartNextIteration());

    sampler->UpdateSamplingPlan(camera->film); //Enable the sampler and SamplingPlanner to post process the rendering result
    reporter.Update();

    reporter.Done();
    LOG(INFO) << "Rendering finished";

    camera->film->WriteImage();
}

Bounds2i SamplerIntegrator::BoundsForTile(const Point2i tile) const
{
    // Compute sample bounds for tile
    int x0 = sampleBounds.pMin.x + tile.x * tileSize;
    int x1 = std::min(x0 + tileSize, sampleBounds.pMax.x);
    int y0 = sampleBounds.pMin.y + tile.y * tileSize;
    int y1 = std::min(y0 + tileSize, sampleBounds.pMax.y);
    Bounds2i tileBounds(Point2i(x0, y0), Point2i(x1, y1));

    return tileBounds;
}

void SamplerIntegrator::RenderTile(const Scene &scene, const Point2i tile) const
{
    MemoryArena arena; // Allocate _MemoryArena_ for tile
    Bounds2i tileBounds = BoundsForTile(tile);
    int seed = sampler->samplingPlanner->currentIteration * tile.y * nTiles.x + tile.x; // Sampler seed is based on index of the tile and iteration -> unique but deterministic
    std::unique_ptr<Sampler> tileSampler = sampler->Clone(seed); // Get sampler instance for tile

    //LOG(INFO) << "Starting image tile " << tileBounds;

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
            CameraSample cameraSample = tileSampler->GetCameraSample(pixel); // Initialize _CameraSample_ for current sample

            // Generate camera ray for current sample
            RayDifferential ray;
            Float rayWeight = camera->GenerateRayDifferential(cameraSample, &ray);
            ray.ScaleDifferentials(1 / std::sqrt((Float)tileSampler->samplingPlanner->PlannedSamplesForThisIteration(pixel)));
            ++nCameraRays;

            Spectrum radiance(0.f);
            if (rayWeight > 0) radiance = Li(ray, scene, *tileSampler, arena); // Evaluate radiance along camera ray

            CheckRadiance(radiance, pixel, tileSampler); // Ensure radiance is within limits and valid

            VLOG(1) << "Camera sample: " << cameraSample << " -> ray: " << ray << " -> radiance = " << radiance;

            filmTile->AddSample(cameraSample.pFilm, radiance, rayWeight); // Add camera ray's contribution to image
            
            arena.Reset(); // Free _MemoryArena_ memory from computing image sample value
        } 
        while (tileSampler->StartNextSample());
    }

    //LOG(INFO) << "Finished image tile " << tileBounds;

    // Merge image tile into _Film_
    camera->film->MergeFilmTile(std::move(filmTile));
}

void SamplerIntegrator::CheckRadiance(Spectrum &radiance, const Point2i pixel, const std::unique_ptr<Sampler> &sampler) const
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

Spectrum SamplerIntegrator::SpecularReflect(const RayDifferential &ray, const SurfaceInteraction &isect,
                                            const Scene &scene, Sampler &sampler, MemoryArena &arena, 
                                            int depth) const 
{
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
    } else
        return Spectrum(0.f);
}

Spectrum SamplerIntegrator::SpecularTransmit(const RayDifferential &ray, const SurfaceInteraction &isect,
                                             const Scene &scene, Sampler &sampler, MemoryArena &arena, 
                                             int depth) const 
{
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
