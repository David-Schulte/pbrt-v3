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
#include "integrators/AdaptiveNLMeansIntegrator.h"

namespace pbrt
{
    void AdaptiveNLMeansIntegrator::Render(const Scene &scene)
    {
        // Initialize values
        Preprocess(scene, *sampler);
        sampleBounds = camera->film->GetSampleBounds();
        sampleExtent = sampleBounds.Diagonal();
        nTiles = Point2i((sampleExtent.x + tileSize - 1) / tileSize,
            (sampleExtent.y + tileSize - 1) / tileSize);

        //Initialize dual buffer by copying the settings from the camera film
        Film *buffer1 = new Film(camera->film); 
        Film *buffer2 = new Film(camera->film);

        ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");

        UniformSampling();

        for (int iteration = 0; iteration < 1; iteration++)
        {
            UpdateSampler();

            ParallelFor2D([&](Point2i tile) // Render section of image corresponding to _tile_
            {
                //Render tiles for each buffer
                RenderTile(scene, tile, buffer1);
                RenderTile(scene, tile, buffer2);

                reporter.Update();
            }, nTiles);
        }

        reporter.Done();
        LOG(INFO) << "Rendering finished";

        //Merge the two films into one and write out the result
        //FilmTile does not carry over the values of the film, as it seems! So merging into the camera via filmTiles like this does nothing!
        //std::unique_ptr<FilmTile> tileOfBuffer1 = buffer1->GetFilmTile(buffer1->croppedPixelBounds);
        //std::unique_ptr<FilmTile> tileOfBuffer2 = buffer2->GetFilmTile(buffer2->croppedPixelBounds);
        //camera->film->MergeFilmTile(std::move(tileOfBuffer1));
        //camera->film->MergeFilmTile(std::move(tileOfBuffer2));

        buffer1->WriteImage();
    }

    void AdaptiveNLMeansIntegrator::UniformSampling()
    {
        //Tell Sampler to create a uniform sample map with x samples per pixel
        //Basically what the samplers do now
    }

    void AdaptiveNLMeansIntegrator::UpdateSampler()
    {
        //Tell Sampler to update the sample map.
        //Requires a dedicated sampler per adaptive integrator!
        //Requires the integrator to know the type of sampler!
    }
}