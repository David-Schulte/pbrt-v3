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

        //Will need two films eventually... either cloning the one from camera, or creating a custom camera that holds 2?
        //Just creating new films here ain't gonna work, because the settings are wrong then...
        Film *film = camera->film; 

        ProgressReporter reporter(nTiles.x * nTiles.y, "Rendering");

        UniformSampling();

        for (int iteration = 0; iteration < 1; iteration++)
        {
            UpdateSampler();

            ParallelFor2D([&](Point2i tile) // Render section of image corresponding to _tile_
            {
                RenderTile(scene, tile, film); //Has to be done twice... (2 films)
                reporter.Update();
            }, nTiles);
        }

        reporter.Done();
        LOG(INFO) << "Rendering finished";

        //Merge the two films into one and write out the result
        film->WriteImage();
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