#pragma once

#include "LPSamplingPlanner.h"

namespace pbrt
{
	void LPSamplingPlanner::UpdateSamplingPlan(Film *film, const int64_t adaptiveSamplesCount)
	{

		int fillMapVal = maxPixelSamplesPerIteration;
		if (adaptiveSamplesCount > 0)
			fillMapVal = adaptiveSamplesCount;

		

		if (firstIteration)
			fillMapVal = initialRenderSamplesPerPixels;

		for (int row = 0; row < plannedSampleMap.size(); row++)
		{
			for (int column = 0; column < plannedSampleMap[0].size(); column++)
			{
				plannedSampleMap[row][column] = fillMapVal;
			}
		}
	}

	void LPSamplingPlanner::CreateSamplingPlan(int samplesPerPixel, Film * film)
	{
		plannedAdaptiveIterations = 1;
		maxPixelSamplesPerIteration = samplesPerPixel;
	}
}