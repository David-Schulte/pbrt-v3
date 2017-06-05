#include "NonAdaptiveSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NonAdaptiveSamplingPlanner::NonAdaptiveSamplingPlanner() {}
    NonAdaptiveSamplingPlanner::~NonAdaptiveSamplingPlanner() {}

    void NonAdaptiveSamplingPlanner::UpdateSampleMap(Film * film)
    {
        FillMapUniformly(sampleMap, maxSamplesPerPixel);
    }

    void NonAdaptiveSamplingPlanner::CreateSamplingPlan(Film *film)
    {
        plannedIterations = 1;
        maxSamplesPerPixel = sampleBudgetPerPixel;
    }

}
