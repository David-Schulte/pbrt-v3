#pragma once

#include "LPSamplingPlanner.h"
#include "film.h"
#include <array>
namespace pbrt
{
	void LPSamplingPlanner::UpdateSamplingPlan(Film *film, const int64_t adaptiveSamplesCount)
	{
		if (firstIteration) //plannedSampleMap is initialized with initialRenderSamplesPerPixels in SamplingPlanner::CreateSampleMap -> we dont have to do anything here for the initial render pass
			return;			//firstIteration is set to false in SamplingPlanner::StartNextIteration

		//copy film after initial Render
		if (initialRenderFilmReady == false)
		{
			copyInitialRenderFilm(film);
			initialRenderFilmReady = true;
		}
		
		//reset plannedSampleMap
		Bounds2i sampleBounds = film->GetSampleBounds();
		Vector2i sampleExtent = sampleBounds.Diagonal();
		plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));

		int fillMapVal = maxPixelSamplesPerIteration; //TODO: HOW to actually get planned sample number?
		if (adaptiveSamplesCount > 0)
			fillMapVal = adaptiveSamplesCount;
		
		//get center pixels
		Point2i margin = computeMargin(film);
		for (int row = (grid.granularity / 2)+2+margin.x; row + grid.granularity/2 < plannedSampleMap.size()-2; row += grid.granularity)
		{
			for (int column = (grid.granularity / 2)+2+margin.y; column + grid.granularity/2 < plannedSampleMap[0].size()-2; column += grid.granularity)
			{
				
				//TODO: Compute linear model here and estimate error of model
				//Add more samples to Area around the center pixels for test purposes
				int kOpt = 180;
				int counter = 0;
				for (int x = row - kOpt / 2; x < row + kOpt / 2; x++)
				{
					for (int y = column - kOpt / 2; y < column + kOpt / 2; y++)
					{
						plannedSampleMap[x][y] = 10;
						counter++;
					}
				}
			}
		}

			//for (int row = 0; row < plannedSampleMap.size(); row++)
			//{
			//	for (int column = 0; column < plannedSampleMap[0].size(); column++)
			//	{
			//		/*Dont add planned samples if:
			//		1: The center pixel is already covered
			//		2. The pixel is outside of the image resolutions (but inside the extent)
			//		*/
			//		if (coverageMask.at(row).at(column).value || row < 2 || row > film->fullResolution.x || column < 2 || column > film->fullResolution.y)
			//		{
			//			plannedSampleMap[row][column] = 0;
			//			continue;
			//		}
			//		
			//		const Float* xyz = initialRenderFilm->GetPixel(Point2i(row-2, column-2)).xyz;

			//		plannedSampleMap[row][column] = fillMapVal;
			//	}
			//}
		

		
	}

	void LPSamplingPlanner::CreateSamplingPlan(int samplesPerPixel, Film * film)
	{
		plannedAdaptiveIterations = 1;
		maxPixelSamplesPerIteration = samplesPerPixel;
	}

	void LPSamplingPlanner::copyInitialRenderFilm(Film* film)
	{
		rawPixelData initValues = rawPixelData();
		initialRenderFilm = std::vector<std::vector<rawPixelData>>(film->fullResolution.x, std::vector<rawPixelData>(film->fullResolution.y, initValues));
		for (int row = 0; row < film->fullResolution.x; row++)
		{
			for (int column = 0; column < film->fullResolution.y; column++)
			{
				initialRenderFilm[row][column] = rawPixelData(film->GetPixel(Point2i(row, column)).xyz);
			}
		}
	}

	Point2i LPSamplingPlanner::computeMargin(Film * film)
	{
		int marginX = film->fullResolution.x % grid.granularity;
		int marginY = film->fullResolution.y % grid.granularity;
	
		return Point2i(marginX/2, marginY/2);
	}
	
}