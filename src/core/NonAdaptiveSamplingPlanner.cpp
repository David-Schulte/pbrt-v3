#include "NonAdaptiveSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NonAdaptiveSamplingPlanner::NonAdaptiveSamplingPlanner() {}
    NonAdaptiveSamplingPlanner::~NonAdaptiveSamplingPlanner() {}

    void NonAdaptiveSamplingPlanner::UpdateSamplingPlan(Film * film)
    {
        FillMapUniformly(maxPixelSamplesPerIteration);
    }

    void NonAdaptiveSamplingPlanner::CreateSamplingPlan(Film *film)
    {
        plannedIterations = 1;
        maxPixelSamplesPerIteration = sampleBudgetPerPixel;
    }

}
