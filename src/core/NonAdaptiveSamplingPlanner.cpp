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
        for (int row = 0; row < sampleMap.size(); row++)
        {
            for (int column = 0; column < sampleMap[0].size(); column++)
            {
                sampleMap[row][column] = maxPixelSamplesPerIteration;
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
