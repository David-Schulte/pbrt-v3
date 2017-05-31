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
        plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y,0));
		currentSampleNumberMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y,0));
		printf("sampleExent: [%i,%i]\n", sampleExtent.x, sampleExtent.y);
		printf("plannedSampleMap X resolution: %i\n", plannedSampleMap.size());
		printf("plannedSampleMap Y resolution: %i\n", plannedSampleMap[0].size());
    }

	void SamplingPlanner::UpdateCurrentSampleNumberMap() 
	{
		for (size_t row = 0; row < currentSampleNumberMap.size(); row++)
		{
			for (size_t column = 0; column < currentSampleNumberMap[0].size(); column++)
			{
				currentSampleNumberMap[row][column] += plannedSampleMap[row][column];
			}
		}
	}

    bool SamplingPlanner::StartNextIteration()
    {
        currentAdaptiveIteration++;

        if (currentAdaptiveIteration > plannedAdaptiveIterations) return false;
        else                                                      return true;
    }

}