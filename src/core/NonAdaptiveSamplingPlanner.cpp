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
        FillMapUniformly(sampleMap, maxTotalSamplesPerPixel);



        if (currentIteration > plannedIterations) //Post processing...
        {
            film->WriteBufferImage("pixelVariance.exr", film->BufferVariance(0));
        }
    }

    void NonAdaptiveSamplingPlanner::CreateSamplingPlan(Film *film)
    {
        film->SetBuffers(1);
        plannedIterations = 1;
        maxTotalSamplesPerPixel = sampleBudgetPerPixel;
    }

}
