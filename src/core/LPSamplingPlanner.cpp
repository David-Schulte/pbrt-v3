#pragma once

#include "LPSamplingPlanner.h"
#include "film.h"
#include <array>
#include <Eigen/Dense>

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
			getPlannedSampleNumber(); // DEBUG! Test matrix inverse.
			printf("\n second render pass \n");
			copyInitialRenderFilm(film);
			initialRenderFilmReady = true;

			//init temporary plannedSampleMap
			Bounds2i sampleBounds = film->GetSampleBounds();
			Vector2i sampleExtent = sampleBounds.Diagonal();
			plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));
			temp_plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));

			//get center pixels
			//grid.margin = computeMargin(film);
			printf("margin: %d %d \n", grid.margin.x, grid.margin.y);
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

		grid.margin = computeMargin(film);
		for (int row = (grid.granularity / 2)+2+ grid.margin.x; row < plannedSampleMap.size()-2; row += grid.granularity)
		{
			for (int column = (grid.granularity / 2)+2+ grid.margin.y; column < plannedSampleMap[0].size()-2; column += grid.granularity)
			{
				//printf("Current center pixel: %d %d \n", row-2, column-2);
				if (coverageMask[row][column].value) //center pixel is already covered
				{
					//printf(" Center pixel already covered.\n\n");
					continue;
				}
				
				

				//TODO: Compute linear model here and estimate error of model
				//Add more samples to Area around the center pixels for test purposes
				int kOpt = 151;
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
	
	int64_t LPSamplingPlanner::getPlannedSampleNumber()
	{


		
		return 0;
	}

	Eigen::MatrixXd constructX(int adaptiveWindowSize, std::vector<std::vector<rawPixelData>> rawPixelData, Point2i centerPixel)
	{
		Eigen::MatrixXd result;

		for (int i = 0; i < adaptiveWindowSize*adaptiveWindowSize; i++)
		{
			result(i,0) = 1;
		}

		for (int i = 0; i < adaptiveWindowSize; i++)
		{
			for (int j = 0; j < adaptiveWindowSize; j++)
			{
				//if (i != j)
				//{
				int row = centerPixel.x - adaptiveWindowSize / 2 + i;
				int column = centerPixel.y - adaptiveWindowSize / 2 + j;
					result(i*adaptiveWindowSize + j, 1) = (rawPixelData[centerPixel.x][centerPixel.y].xyz[0] - rawPixelData[row][column].xyz[0] 
														+ rawPixelData[centerPixel.x][centerPixel.y].xyz[1] - rawPixelData[row][column].xyz[1]
														+ rawPixelData[centerPixel.x][centerPixel.y].xyz[2] - rawPixelData[row][column].xyz[2]) / 3.0;
					// Not sure here!
					//result(i*adaptiveWindowSize + j, 2) = rawPixelData[centerPixel.x][centerPixel.y].xyz[1] - rawPixelData[i][j].xyz[1];
					//result(i*adaptiveWindowSize + j, 3) = rawPixelData[centerPixel.x][centerPixel.y].xyz[2] - rawPixelData[i][j].xyz[2];
				//}
			}
		}

		return result;
	}

	Eigen::VectorXd constructY(int adaptiveWindowSize, std::vector<std::vector<rawPixelData>> rawPixelData, Point2i centerPixel)
	{
		Eigen::VectorXd result;
		for (int i = 0; i < adaptiveWindowSize; i++)
		{
			for (int j = 0; j < adaptiveWindowSize; j++)
			{
				//if (i != j)
				//{
				int row = centerPixel.x - adaptiveWindowSize / 2 + i;
				int column = centerPixel.y - adaptiveWindowSize / 2 + j;
				result(i*adaptiveWindowSize + j) = (rawPixelData[centerPixel.x][centerPixel.y].xyz[0] - rawPixelData[row][column].xyz[0]
												+ rawPixelData[centerPixel.x][centerPixel.y].xyz[1] - rawPixelData[row][column].xyz[1]
												+ rawPixelData[centerPixel.x][centerPixel.y].xyz[2] - rawPixelData[row][column].xyz[2]) / 3.0;
				// Not sure here!
				//result(i*adaptiveWindowSize + j, 2) = rawPixelData[centerPixel.x][centerPixel.y].xyz[1] - rawPixelData[i][j].xyz[1];
				//result(i*adaptiveWindowSize + j, 3) = rawPixelData[centerPixel.x][centerPixel.y].xyz[2] - rawPixelData[i][j].xyz[2];
				//}
			}
		}
		return result;
	}

	void testMatrix() 
	{
		Eigen::MatrixXd m(19, 19);

		double test[] = { -34.7622, -23.6197, 40.2716, -14.6841, 42.9386, 8.7045, -24.1935, 7.8525, -16.4643, 10.9867, 2.1650, 30.3364, -44.7323, 6.1200, -24.8194, -32.1234, -39.4371, 17.1808, -17.7528,
			32.5817, -35.4461, 44.4787, 32.1194, 27.5713, -29.2258, -9.1280, -26.2716, 17.9728, 11.7666, -40.3270, -43.9529, 23.7858, 38.1867, -20.9559, -7.7114, 11.0959, 19.5140, 28.4739,
			3.8342, -36.3931, -0.9136, -48.4597, -1.3208, -19.8754, 9.4896, -4.1151, -36.3447, 35.9442, 31.8149, -10.0742, -23.0881, 16.9175, 11.7091, -40.5771, 27.8802, -43.2007, -2.8643,
			49.6135, 36.9292, -1.0747, -45.6976, -6.4141, -2.9077, -23.7788, 46.3089, 22.1227, 30.5489, 31.7547, 2.6876, -7.7164, -30.9567, -23.4719, 9.8524, -7.6547, -24.5210, -46.4237,
			-42.1824, 7.9705, -16.2281, -33.1010, -5.3216, -26.9512, 10.2843, 4.6806, -39.3238, 7.6722, 22.2440, -8.3201, 4.7871, -13.1083, 32.4376, -2.9076, -40.9177, -27.5960, -32.4126,
			-5.7322, 4.9860, 40.0054, 14.9115, -19.3651, 34.4309, 21.1216, 2.1136, 15.3757, -31.7078, -35.0135, 15.6860, 44.2737, -3.9274, 48.2663, 19.5949, -23.3529, 16.7833, 22.1758,
			-39.3347, -35.5045, -13.0753, 23.1722, 0.8509, -30.5236, -27.8253, -26.8406, -0.5826, -26.0068, 15.9605, 12.7973, -8.2256, 48.1638, 23.0249, 19.9888, -34.6343, 34.4392, -2.6514,
			46.1898, 35.3031, -38.8797, 14.7746, 1.0772, -27.4078, -38.2582, -1.1102, 27.9052, 38.6512, 1.8595, -20.8016, 48.3052, -34.3595, -15.6123, 13.8531, -21.8995, -15.5538, -34.7279,
			-49.5366, 12.2055, 28.0252, -4.9076, 31.7628, -32.9292, -20.3324, 12.4060, 21.5037, -47.1326, 47.2975, -6.8349, -19.8545, 35.5523, 8.4069, -46.6396, -5.9915, 28.0520, -15.8875,
			27.4910, -14.9048, -11.0261, 4.7009, 29.4831, -27.2336, -18.1222, 17.9136, 40.3721, -1.0099, 14.8991, -48.4513, 20.1099, 14.4765, -39.2231, -43.1194, 2.7143, 17.5332, 10.7389,
			31.7303, 1.3250, -25.8309, -20.3679, 14.4318, -6.4301, -7.5833, -10.4485, 39.0923, -33.2073, 30.0331, 48.4064, 16.6339, -12.3728, 40.6308, -18.0400, -4.2576, -49.3285, -30.8255,
			36.8695, -9.8192, -9.6088, 24.4693, -12.1391, -18.8898, 0.7858, -13.2563, -16.5837, 47.8681, -4.6202, -33.2832, 3.9126, -30.9076, 37.9654, 3.0864, 37.5372, 10.2170, 23.8427,
			-41.5564, -42.4033, -40.3545, -31.1045, 31.1580, 42.3380, -41.4484, 48.7982, 19.8746, 21.2694, -6.7608, -39.3784, 19.8106, -7.1747, 31.7761, 15.4446, 1.8052, -11.3229, -25.7150,
			-10.0217, -26.0084, -36.8027, 18.6775, 3.2826, -6.9793, -23.7518, -46.2261, -30.2190, 0.0472, 32.5314, -12.7590, 16.6528, -1.7978, -23.9272, -9.2381, 44.3623, 41.5991, 41.7424,
			-24.0130, -37.6681, 44.2051, -31.6489, -14.9273, -31.5184, 30.1015, 38.5168, -46.9459, -2.8912, -41.6530, -30.1882, -32.1868, -37.9388, 9.4356, 31.9981, 13.7709, -49.8849, -23.0938,
			30.0068, -31.6092, 45.6135, -13.1515, 43.9002, 40.4881, -47.0780, 41.3287, 24.4074, -44.0381, -36.6829, -1.0312, -37.1986, 8.9507, -47.7487, 21.8359, 45.7694, -3.7551, 26.5500,
			-6.8586, -26.0047, 7.5209, 12.5619, 37.5943, 47.9748, 42.8854, 29.6184, 0.0022, 18.1972, -32.6611, -16.0507, 49.9080, -27.3812, -7.4741, 46.8649, -25.9293, -7.5651, -31.1338,
			41.0648, -8.2733, -44.0220, 28.0227, 5.0156, -6.1130, 23.0331, -40.1288, -2.0078, -45.7569, -10.9062, 45.1630, -32.8879, -11.5381, -18.7281, 3.1334, 17.6122, -3.9084, -21.2502,
			-31.8153, -45.0346, -26.5220, -41.8874, 12.2475, -38.8881, -1.1391, -23.8129, 40.4722, -42.8555, 33.1380, 42.0332, -46.7399, 8.2986, -33.8515, -17.4854, -21.0935, 27.0160, -40.8887 };

		for (int i = 0; i < 19; i++)
		{
			for (int j = 0; j < 19; j++)
			{
				m(i, j) = (double)test[i * 19 + j];
			}
		}
		std::cout << "M: \n" << m << std::endl;
		std::cout << "Inverse M: \n" << m.inverse() << std::endl;
	}
}