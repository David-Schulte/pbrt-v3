#include "NonAdaptiveSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NonAdaptiveSamplingPlanner::NonAdaptiveSamplingPlanner() {}
    NonAdaptiveSamplingPlanner::~NonAdaptiveSamplingPlanner() {}

    void NonAdaptiveSamplingPlanner::UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount)
    {
        for (int row = 0; row < plannedSampleMap.size(); row++)
        {
            for (int column = 0; column < plannedSampleMap[0].size(); column++)
            {
                plannedSampleMap[row][column] = maxPixelSamplesPerIteration;
            }
        }
		printf("UPDATESAMPLEPLANNER!\n");
    }

    void NonAdaptiveSamplingPlanner::CreateSamplingPlan(int samplesPerPixel, Film *film)
    {
        plannedAdaptiveIterations = 1;
        maxPixelSamplesPerIteration = samplesPerPixel;
		printf("CREATESAMPLINGPLAN!\n");
    }

}
