#include "SamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt 
{

    SamplingPlanner::SamplingPlanner() {}
    SamplingPlanner::~SamplingPlanner() {}

    void SamplingPlanner::InitializeSamplingPlan(int samplesPerPixel, Film * film)
    {
        currentAdaptiveIteration = 1;

        CreateSampleMap(film);
        CreateSamplingPlan(samplesPerPixel, film);
    }

    void SamplingPlanner::CreateSampleMap(Film * film)
    {
        Bounds2i sampleBounds = film->GetSampleBounds();
        Vector2i sampleExtent = sampleBounds.Diagonal();
        sampleMap = std::vector<std::vector<int>>(sampleExtent.x, std::vector<int>(sampleExtent.y));
    }

    bool SamplingPlanner::StartNextIteration()
    {
        currentAdaptiveIteration++;

        if (currentAdaptiveIteration > plannedAdaptiveIterations) return false;
        else                                                      return true;
    }

}