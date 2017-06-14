#pragma once

#include "LPSamplingPlanner.h"
#include "film.h"
#include <array>
namespace pbrt
{
	void LPSamplingPlanner::UpdateSamplingPlan(Film *film, const int64_t adaptiveSamplesCount)
	{
		static int itCounter = 0;
		
		if (firstIteration) //plannedSampleMap is initialized with initialRenderSamplesPerPixels in SamplingPlanner::CreateSampleMap -> we dont have to do anything here for the initial render pass
			return;			//firstIteration is set to false in SamplingPlanner::StartNextIteration

		//copy film after initial Render
		if (initialRenderFilmReady == false)
		{
			printf("\n second render pass \n");
			copyInitialRenderFilm(film);
			initialRenderFilmReady = true;

			//init temporary plannedSampleMap
			Bounds2i sampleBounds = film->GetSampleBounds();
			Vector2i sampleExtent = sampleBounds.Diagonal();
			plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));
			temp_plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));

		}
		itCounter++;
		
		//plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));

		//int fillMapVal = maxPixelSamplesPerIteration; //TODO: HOW to actually get planned sample number?
		//if (adaptiveSamplesCount > 0)
		//	fillMapVal = adaptiveSamplesCount;

		//all pixels covered --> use actual plannedSampleMap
		if (numberCoveredPixels == film->fullResolution.x * film->fullResolution.y)
		{
			printf("\n all pixels covered \n");
			plannedSampleMap = temp_plannedSampleMap;
			finalRender = true;
			return;
		}

		
		//get center pixels
		Point2i margin = computeMargin(film);
		printf("margin: %d %d \n", margin.x, margin.y);
		for (int row = (grid.granularity / 2)+2+margin.x; row < plannedSampleMap.size()-2; row += grid.granularity)
		{
			for (int column = (grid.granularity / 2)+2+margin.y; column < plannedSampleMap[0].size()-2; column += grid.granularity)
			{
				//printf("Current center pixel: %d %d \n", row-2, column-2);
				if (coverageMask[row][column].value) //center pixel is already covered
				{
					//printf(" Center pixel already covered.\n\n");
					continue;
				}
				
				

				//TODO: Compute linear model here and estimate error of model
				//Add more samples to Area around the center pixels for test purposes
				int kOpt = 10;
				for (int x = row - kOpt / 2; x <= row + kOpt / 2; x++)
				{
					for (int y = column - kOpt / 2; y <= column + kOpt / 2; y++)
					{
						//printf("Current pixel in kOpt window: % d %d \n .", x-2, y-2);
						if (x < 2 || x > plannedSampleMap.size() - 1 - 2 || y < 2 || y > plannedSampleMap[0].size() - 1 - 2)
						{
							//printf("x or y outside of image \n");
							continue; //ignore pixels that are not part of the real image
						}
							

						//for pixels that are part of the image but the kOpt window reaches over the border -> add coverage but dont add additional samples
						if (row - kOpt / 2 < 2 || row + kOpt / 2 > plannedSampleMap.size() - 3 || column - kOpt / 2 < 2 || column + kOpt / 2 > plannedSampleMap[0].size() - 3)
						{
							//printf("x or y inside image but kOpt window outside \n");
							if (coverageMask[x][y].value)
							{
								//printf("xy already covered \n");
								continue;
							}
							
							numberCoveredPixels++;
							coverageMask[x][y].value = true;
							//printf("xy now covered but did not get any samples \n");
							continue;
						}
					

						if (coverageMask[x][y].value == false)
						{
						//	printf("xy now covered and got samples \n");
			
							temp_plannedSampleMap[x][y] += 10;
							
							
							numberCoveredPixels++;
							coverageMask[x][y].value = true;
							//printf("\n number covered pixels: %d \n", numberCoveredPixels);
						}
						else
						{
							//printf("xy already covered \n");
							//printf("\n pixel covered by multiple models \n");
							//TODO: what to do if a pixel is covered by multiple linear models?
						}
					}
				}
			}
		}
		grid.refineGrid();
		
		
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

	bool LPSamplingPlanner::StartNextIteration()
	{
		SamplingPlanner::StartNextIteration();
		return finalRender ? false : true;
	}
	
}