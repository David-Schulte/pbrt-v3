#pragma once

#include "LPSamplingPlanner.h"
#include "imageio.h"
#include "film.h"
#include <array>
#include <math.h>

namespace pbrt
{
	void LPSamplingPlanner::UpdateSamplingPlan(Film *film, const int64_t adaptiveSamplesCount)
	{
		//prediction windows of size fixedWindowSize x fixedWindowSize might reach over the border. 
		//This parameter specifies how the algorithm should behave in such cases near the border.
		//if this parameter is false, no linear models will be computed in such cases.
		//If this parameter is true the algorithm will use the predictions windows that do not reach over the border in such cases. 
		useSmallWindowsOnBorder = true;
		

		if (firstIteration) //plannedSampleMap is initialized with initialRenderSamplesPerPixels in SamplingPlanner::CreateSampleMap -> we dont have to do anything here for the initial render pass
			return;			//firstIteration is set to false in SamplingPlanner::StartNextIteration

		//copy film after initial Render and initialize variables for adaptive iterations
		if (initialRenderFilmReady == false)
		{
			initialRenderFilmReady = true;
			copyInitialRenderFilm(film);
			initForAdaptiveIterations(film);
			printf("\n initial Render finished\n");
		}

		//all pixels covered --> use actual plannedSampleMap
		if (allPixelsCoveredByAtLeastOneLinearModel(film))
		{
			printf("\n\n MinMaxSamples send %d, %d \n", minSamplesSend, maxSamplesSend);
			printf("\n\n MinMaxError send %.12f, %.12f \n", minError, maxError);

			printf("\n all pixels covered \n");
			averagePlannedSampleNumber(film);
			plannedSampleMap = temp_plannedSampleMap;
			
			currentLevelOfAdaptation++;
			if (currentLevelOfAdaptation == maxLevelOfAdaptation)
			{
				pbrt::Bounds2i sampleBounds = film->GetSampleBounds();
				outPutVisualizationImages(film->filename, sampleBounds, film->fullResolution);
				finalRender = true; //used in LPSamplingPlanner::StartNextIteration to determine if this is the final rendering before writing the image to a file 
			}
			else
			{
				initialRenderFilmReady = false; //reset to copy film anew in next adaptation level
				printf("\n adaptive level %d finished\n", currentLevelOfAdaptation);
			}
			return;
		}

		//iterate over center pixels that lie on a regular grid
		for (int32_t centerPixelX = (grid.granularity / 2) + filmExtentResDiff / 2 + grid.margin.x; centerPixelX < plannedSampleMap.size() - filmExtentResDiff / 2; centerPixelX += grid.granularity)
		{
			for (int32_t centerPixelY = (grid.granularity / 2) + filmExtentResDiff / 2 + grid.margin.y; centerPixelY < plannedSampleMap[0].size() - filmExtentResDiff / 2; centerPixelY += grid.granularity)
			{
				LinearModel minErrorLinModel;
				int64_t kOpt = grid.fixedWindowSize;

				if (coverageMask[centerPixelX][centerPixelY].value) //center pixel is already covered --> skip this center pixel
					continue;
				
					//Compute linear models if their prediction window does not reach over the border
					std::vector<LinearModel> linModels = computeAllLinearModels(centerPixelX, centerPixelY);
					minErrorLinModel = (linModels.size() > 0) ? linModels[findMinErrorLinModelIdx(linModels)] : LinearModel();
					kOpt = (linModels.size() > 0) ? minErrorLinModel.windowSize : grid.fixedWindowSize;
				

				//iterate over all pixels within a prediction window of size kOpt x kOpt
				for (int32_t x = centerPixelX - kOpt / 2; x <= centerPixelX + kOpt / 2; x++)
				{
					for (int32_t y = centerPixelY - kOpt / 2; y <= centerPixelY + kOpt / 2; y++)
					{
						if (!isPixelPartOfImage(pbrt::Point2i(x, y)))
							continue; //ignore pixels that are not part of the real image. Possible if there is an extend > image resolution
					
						//kOpt window reaches over the border -> add coverage but do not add additional samples
						if(windowReachesOverBorder(pbrt::Point2i(centerPixelX, centerPixelY), kOpt))
						{
							if (coverageMask[x][y].value)
								continue; // pixel already covered
							
							numberCoveredPixels++;
							coverageMask[x][y].value = true;
							coverageMask[x][y].coverageCounter++;
							continue;
						}

						//pixel not covered yet and prediction windows does not reach over border
						if (coverageMask[x][y].value == false)
						{
							numberCoveredPixels++;
							coverageMask[x][y].value = true;
						}

						temp_plannedSampleMap[x][y] += getPlannedSampleNumber(minErrorLinModel, pbrt::Point2i(x, y), 1);
						coverageMask[x][y].coverageCounter++; 
						initialRenderFilm[x - filmExtentResDiff/2][y - filmExtentResDiff/2].predError += minErrorLinModel.predError;
					}
				}
			}
		}
		grid.refineGrid(); //refine grid to create center pixels that lie between the current center pixels in the next iteration
	}

	void LPSamplingPlanner::CreateSamplingPlan(int samplesPerPixel, Film * film)
	{
		plannedAdaptiveIterations = 1;
		maxPixelSamplesPerIteration = samplesPerPixel;
	}

	Float clamp(Float n, Float lower, Float upper) 
	{
		return std::max(lower, std::min(n, upper));
	}

	void LPSamplingPlanner::copyInitialRenderFilm(Film* film)
	{
		rawPixelData initValues = rawPixelData();
		initialRenderFilm = std::vector<std::vector<rawPixelData>>(film->fullResolution.x, std::vector<rawPixelData>(film->fullResolution.y, initValues));

		Float maxColorChannelRGBVal = std::numeric_limits<Float>::min();
		Float maxColorChannelXYZVal = std::numeric_limits<Float>::min();

		Float* rgb = new Float[3];

		for (int row = 0; row < film->fullResolution.x; row++)
		{
			for (int column = 0; column < film->fullResolution.y; column++)
			{

				XYZToRGB(film->GetPixel(Point2i(row, column)).xyz, &rgb[0], true);
				// Normalize pixel with weight sum
				Float filterWeightSum = film->GetPixel(Point2i(row, column)).filterWeightSum;
				if (filterWeightSum != 0) {
					Float invWt = (Float)1 / filterWeightSum;
					//rgb[0] = std::max((Float)0, rgb[0] * invWt);
					//rgb[1] = std::max((Float)0, rgb[1] * invWt);
					//rgb[2] = std::max((Float)0, rgb[2] * invWt);

					rgb[0] = std::min((Float)1, std::max((Float)0, rgb[0] * invWt));
					rgb[1] = std::min((Float)1, std::max((Float)0, rgb[1] * invWt));
					rgb[2] = std::min((Float)1, std::max((Float)0, rgb[2] * invWt));
				}

				initialRenderFilm[row][column].rgb[0] = rgb[0];
				initialRenderFilm[row][column].rgb[1] = rgb[1];
				initialRenderFilm[row][column].rgb[2] = rgb[2];
			}
		}
		delete[] rgb;
	}

	Point2i LPSamplingPlanner::computeMargin(Film * film)
	{
		int marginX = film->fullResolution.x % grid.granularity;
		int marginY = film->fullResolution.y % grid.granularity;
	
		return Point2i(marginX/2, marginY/2);
	}

	void LPSamplingPlanner::initForAdaptiveIterations(Film * film)
	{

		//init variables
		numberCoveredPixels = 0;
		grid = AdaptiveGrid();

		//init all maps
		Bounds2i sampleBounds = film->GetSampleBounds();
		Vector2i sampleExtent = sampleBounds.Diagonal();
		plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));
		temp_plannedSampleMap = std::vector<std::vector<int64_t>>(sampleExtent.x, std::vector<int64_t>(sampleExtent.y, 0));
		coverageMask = std::vector<std::vector<vector_bool>>(sampleExtent.x, std::vector<vector_bool>(sampleExtent.y, vector_bool()));

		//compute intial margin for horizontally and vertically centered center pixels
		grid.margin = computeMargin(film);
	}

	bool LPSamplingPlanner::allPixelsCoveredByAtLeastOneLinearModel(Film * film)
	{
		return (numberCoveredPixels == film->fullResolution.x * film->fullResolution.y);
	}

	void LPSamplingPlanner::averagePlannedSampleNumber(Film* film)
	{
		Float averageSPP = 0;
		for (int row = 0; row < temp_plannedSampleMap.size(); row++)
		{
			for (int column = 0; column < temp_plannedSampleMap[0].size(); column++)
			{
				if (coverageMask[row][column].coverageCounter == 0)
				{
					temp_plannedSampleMap[row][column] = 0;
				}
				else
				{
					temp_plannedSampleMap[row][column] /= coverageMask[row][column].coverageCounter;
					averageSPP += Float(temp_plannedSampleMap[row][column]);
					if (row >= filmExtentResDiff && column >= filmExtentResDiff)
					{
						initialRenderFilm[row-filmExtentResDiff/2][column-filmExtentResDiff/2].predError /= coverageMask[row][column].coverageCounter;
					}
				}
			}
		}
		averageSPP /= Float(film->fullResolution.x * film->fullResolution.y);
		printf("\n Average samples per pixel (averageSPP): %f\n", averageSPP);
	}

	bool LPSamplingPlanner::isPixelPartOfImage(pbrt::Point2i pixel)
	{
		return !(pixel.x < filmExtentResDiff / 2 || pixel.x > plannedSampleMap.size() - 1 - filmExtentResDiff / 2 || pixel.y < filmExtentResDiff / 2 || pixel.y > plannedSampleMap[0].size() - 1 - filmExtentResDiff / 2);
	}

	bool LPSamplingPlanner::windowReachesOverBorder(pbrt::Point2i centerPixel, int32_t windowSize)
	{
		pbrt::Point2i leftUpperCorner = pbrt::Point2i(centerPixel.x - windowSize / 2, centerPixel.y - windowSize / 2);
		pbrt::Point2i rightLowerCorner = pbrt::Point2i(centerPixel.x + windowSize / 2, centerPixel.y + windowSize / 2);

		return (!isPixelPartOfImage(leftUpperCorner) || !isPixelPartOfImage(rightLowerCorner));
	}

	std::vector<LinearModel> LPSamplingPlanner::computeAllLinearModels(int32_t centerPixelX, int32_t centerPixelY)
	{
		std::vector<LinearModel> linModels;
		// Compute linear models.
		for (int32_t adaptiveWindowSize = 3; adaptiveWindowSize < grid.fixedWindowSize + 1; adaptiveWindowSize += 2)
		{
			int32_t windowSize = useSmallWindowsOnBorder ? adaptiveWindowSize : grid.fixedWindowSize; //check if algorithm should compute linear models for pixels near the border (if not all linear models can be computed)
			if (windowReachesOverBorder(pbrt::Point2i(centerPixelX, centerPixelY), windowSize)) //if windowSize = grid.fixedWindowSize this will be false for pixels near the border an no linear models will be computed
			{
				break;
			}
			LinearModel prevModel = linModels.empty() ? LinearModel() : linModels.back();
			Point2i centerPixel = Point2i(centerPixelX - (filmExtentResDiff / 2), centerPixelY - (filmExtentResDiff / 2));
			LinearModel currentModel = computeLinearModelAndPredictionError(prevModel, adaptiveWindowSize, initialRenderFilm, centerPixel);
			allCenterPixel.push_back(centerPixel);
			linModels.push_back(currentModel);
		}
		return linModels;
		
	}

	bool LPSamplingPlanner::StartNextIteration()
	{
		SamplingPlanner::StartNextIteration();
		return finalRender ? false : true;
	}
	
	int64_t LPSamplingPlanner::getPlannedSampleNumber(LinearModel minErrorLinModel, const pbrt::Point2i pixel, int32_t additionalSampleStep)
	{
		
		int64_t plannedSampleNumber = 0;
		double threshHold = 0.0000000001;
		int32_t receivedSamples = CurrentSampleNumber(pixel);
		double predError = minErrorLinModel.predError;

		/*printf("\n\n current pixel: %d, %d\n", pixel.x, pixel.y);
		printf("\n error: %0.12f\n", predError);
		printf("\n receivedSamples: %d\n", receivedSamples);*/

		//Float fallOffFactor = predError/ minErrorLinModel.predError;

		while (predError > threshHold)
		{	
			double reductionFactor = std::pow(static_cast<double>(receivedSamples), ((double)-4 / (double)(featureDim + 4)));
			predError = predError * reductionFactor;
			receivedSamples++;

			plannedSampleNumber+= additionalSampleStep;
		}
		/*printf("\n Planned Samples: %d\n", plannedSampleNumber);*/

		//DEBUG
		minSamplesSend = (plannedSampleNumber < minSamplesSend) ? plannedSampleNumber : minSamplesSend;
		maxSamplesSend = (plannedSampleNumber > maxSamplesSend) ? plannedSampleNumber : maxSamplesSend;

		minError = (minErrorLinModel.predError < minError) ? minErrorLinModel.predError : minError;
		maxError = (minErrorLinModel.predError > maxError) ? minErrorLinModel.predError : maxError;
		return plannedSampleNumber;
	}

	Eigen::MatrixXd constructXc(int adaptiveWindowSize, int featureDim = 2)
	{
		Eigen::MatrixXd result(adaptiveWindowSize*adaptiveWindowSize, 1 + featureDim);

		for (int i = 0; i < adaptiveWindowSize*adaptiveWindowSize; i++)
		{
			result(i, 0) = 1;
		}

		for (int featureIdx = 0; featureIdx < featureDim; featureIdx++)
		{
			for (int row = 0; row < adaptiveWindowSize; row++)
			{
				for (int column = 0; column < adaptiveWindowSize; column++)
				{
					Eigen::VectorXd feature(featureDim);
					feature<<(Float)row, (Float)column;
					result(row * adaptiveWindowSize + column, featureIdx+1) = feature(featureIdx) - adaptiveWindowSize/2;
				}
			}
		}

		return result;
	}

	Eigen::VectorXd constructYc(int adaptiveWindowSize, const std::vector<std::vector<rawPixelData>>& rawPixelData, Point2i centerPixel)
	{
		Eigen::VectorXd result(adaptiveWindowSize*adaptiveWindowSize,1);
		for (int i = 0; i < adaptiveWindowSize; i++)
		{
			for (int j = 0; j < adaptiveWindowSize; j++)
			{
				int row = centerPixel.y - adaptiveWindowSize / 2 + i;
				int column = centerPixel.x - adaptiveWindowSize / 2 + j;
				result(i*adaptiveWindowSize + j) = (rawPixelData[column][row].rgb[0]
												+ rawPixelData[column][row].rgb[1]
												+ rawPixelData[column][row].rgb[2]) / 3.0;
			}
		}
		return result;
	}

	LinearModel LPSamplingPlanner::computeLinearModelAndPredictionError(const LinearModel previousLinModel, const int adaptiveWindowSize, const std::vector<std::vector<rawPixelData>>& rawPixelData, const Point2i centerPixel)
	{
		LinearModel result;

		result.center = centerPixel;
		result.windowSize = adaptiveWindowSize;

		Eigen::MatrixXd X = constructXc(adaptiveWindowSize, featureDim);
		Eigen::VectorXd Y = constructYc(adaptiveWindowSize, rawPixelData, centerPixel);

		Eigen::MatrixXd A = X.transpose()*X;
		Eigen::VectorXd B = X.transpose()*Y;

		Eigen::MatrixXd augmentedA(A.rows(), A.cols() + B.cols());
		augmentedA << A, B;
		
		//result.linModelCoeffs = Eigen::VectorXd(adaptiveWindowSize, Y.cols());
		Eigen::FullPivLU<Eigen::MatrixXd> LU_A = A.fullPivLu();
		////if (LU_A.isInvertible() || adaptiveWindowSize <= 3)
		if (A.fullPivLu().rank() == augmentedA.fullPivLu().rank())
		{
			result.linModelCoeffs = LU_A.solve(B);

			// Update prediction error of the linear model here.
			updatePredictionErrorEstimate(result, previousLinModel, rawPixelData, X, Y);
		}
		else 
		{
			result.linModelCoeffs = Eigen::VectorXd::Zero(X.cols(),1);
			result.predError = 0.0;
		}
		//result.print();
		
		//result.linModelCoeffs = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
		//result.linModelCoeffs = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
		//updatePredictionErrorEstimate(result, previousLinModel, rawPixelData, X, Y);

		return result;
	}

	void LPSamplingPlanner::updatePredictionErrorEstimate(LinearModel& linModel, const LinearModel previousLinModel, const std::vector<std::vector<rawPixelData>>& rawPixelData, const Eigen::MatrixXd Xc, const Eigen::MatrixXd Yc)
	{
		std::vector<Float> linModelErrors;
		Eigen::MatrixXd XcT;

		Float linModelError;
		Float newLinModelError;

		int windowSize = linModel.windowSize;

			if (windowSize == 3)
			{
				linModelError = 0;
				XcT = Xc.transpose();
				if ((Xc.transpose()*Xc).determinant() != 0.0)
				{
					Eigen::MatrixXd Pc = (Xc.transpose()*Xc).inverse();
					for (int i = 0; i < 9; i++)
					{
						Float nominator = ((linModel.linModelCoeffs.transpose() * Xc.row(i).transpose())(0, 0) - Yc(i));
						Float denominator = (1.0 - (Xc.row(i) * Pc * Xc.row(i).transpose())(0, 0)) + std::numeric_limits<Float>::min();

						newLinModelError = nominator / (denominator);
						newLinModelError *= newLinModelError;
						linModelError += newLinModelError;
						linModel.nominatorPredError = linModelError;
					}
				}
				else { linModelError = 0; }
			}
			else
				if (windowSize > 3)
				{
					XcT = Xc.transpose();
					linModelError = previousLinModel.nominatorPredError;
					int r = (windowSize / 2);

					for (int i = 0; i < (windowSize * windowSize - 1); i++)
					{
						if (i < windowSize || i % windowSize == 0 || i % windowSize == windowSize - 1 || i >= (windowSize * windowSize) - windowSize)
						{
							Float tmpLinModelError = (previousLinModel.linModelCoeffs.transpose()*Xc.row(i).transpose())(0, 0) - Yc(i);
							newLinModelError += (tmpLinModelError*tmpLinModelError);
						}
					}
					linModelError += newLinModelError;
					linModel.nominatorPredError = linModelError;
					linModelError /= (Float)((2 * r + 1) * (2 * r + 1));
				}
		linModel.predError = linModelError;
	}

	int LPSamplingPlanner::findMinErrorLinModelIdx(std::vector<LinearModel> linModels)
	{
		Float minLinModelError = std::numeric_limits<Float>::max();
		int minLinModelErrorIdx = 0;

		for (int i = 0; i < linModels.size(); i++)
		{
			if (linModels[i].predError <= minLinModelError)
			{
				minLinModelError = linModels[i].predError;
				minLinModelErrorIdx = i;
			}
		}

		/*printf("\n//////////////////////////////////////////////////////////////////////////////\n");
		printf("====Min error linear model [window size , prediction error , [center.x , center.y]]: [%d , %f , [%d , %d] ]\n", linModels[minLinModelErrorIdx].windowSize, linModels[minLinModelErrorIdx].predError, linModels[minLinModelErrorIdx].center.x, linModels[minLinModelErrorIdx].center.y);
		printf("//////////////////////////////////////////////////////////////////////////////\n");*/

		return minLinModelErrorIdx;
	}

	void writeVisualizationToImage(std::function<void(Float* arrayToFill)> fillRGBFunc, const std::string visualizationName, const std::string fileName, Bounds2i sampleBounds, Point2i fullResolution, Float initValR, Float initValG, Float initValB)
	{
		Vector2<int> sampleExtent = sampleBounds.Diagonal();

		Float *visualizationImageRGB = new Float[sampleExtent.x*sampleExtent.y * 3];

		// Initialize all values
		for (int row = 0; row < sampleExtent.x; row++)
		{
			for (int column = 0; column < sampleExtent.y; column++)
			{
				visualizationImageRGB[3 * (column * sampleExtent.x + row) + 0] = initValR;
				visualizationImageRGB[3 * (column * sampleExtent.x + row) + 1] = initValG;
				visualizationImageRGB[3 * (column * sampleExtent.x + row) + 2] = initValB;
			}
		}

		fillRGBFunc(visualizationImageRGB);

		std::string name = fileName;
		name = visualizationName + name;

		WriteImage(name, &visualizationImageRGB[0], sampleBounds, fullResolution);

		delete[] visualizationImageRGB;
	}

	int numberOfNMaxVal = 75;
	Float scaleFactorErrorMap = 7.0;
	Float scaleFactorSampleMap = 1.05;

	void LPSamplingPlanner::outPutVisualizationImages(const std::string fileName, Bounds2i sampleBounds, Point2i fullResolution)
	{
		/////////////////////////////////////////////////////////////////////////////////////////
		// Visualize all centerPixel
		Vector2i sampleExtent = sampleBounds.Diagonal();

		writeVisualizationToImage([&](Float* arrayToFill)
		{
			for (int allCenterPixelIdx = 0; allCenterPixelIdx < allCenterPixel.size(); allCenterPixelIdx++)
			{
				int column = allCenterPixel[allCenterPixelIdx].y;
				int row = allCenterPixel[allCenterPixelIdx].x;
				arrayToFill[3 * (column * sampleExtent.x + row) + 0] = 1.0;
				arrayToFill[3 * (column * sampleExtent.x + row) + 1] = 0.0;
				arrayToFill[3 * (column * sampleExtent.x + row) + 2] = 0.0;
			}
		},
		"AllCenterPixel_", fileName, sampleBounds, fullResolution, 1.0, 1.0, 1.0);

		/////////////////////////////////////////////////////////////////////////////////////////
		// Visualize sample map

		writeVisualizationToImage([&](Float* arrayToFill)
		{
			int64_t sampleMapMaxVal = 0;
			for (int row = 0; row < sampleExtent.x; row++)
			{
				for (int column = 0; column < sampleExtent.y; column++)
				{
					if (sampleMapMaxVal < plannedSampleMap[row][column])
					{
						sampleMapMaxVal = plannedSampleMap[row][column];
					}
				}
			}

			for (int row = 0; row < sampleExtent.x; row++)
			{
				for (int column = 0; column < sampleExtent.y; column++)
				{
					arrayToFill[3 * (column * sampleExtent.x + row) + 0] = 1.0;
					arrayToFill[3 * (column * sampleExtent.x + row) + 1] = (1.0 - std::min((Float)1.0, (Float)plannedSampleMap[row][column] / (Float)sampleMapMaxVal * scaleFactorSampleMap));
					arrayToFill[3 * (column * sampleExtent.x + row) + 2] = (1.0 - std::min((Float)1.0, (Float)plannedSampleMap[row][column] / (Float)sampleMapMaxVal * scaleFactorSampleMap));
				}
			}
		},
		"SampleMap_", fileName, sampleBounds, fullResolution, 1.0, 1.0, 1.0);

		/////////////////////////////////////////////////////////////////////////////////////////
		// Visualize estimated error (color ramp: blue to red)

		writeVisualizationToImage([&](Float* arrayToFill)
		{
			int resX = fullResolution.x;
			int resY = fullResolution.y;

				Float errorMapMaxVal = 0;
			for (int row = 0; row < resX; row++)
			{
				for (int column = 0; column < resY; column++)
				{
					if (errorMapMaxVal < initialRenderFilm[row][column].predError)
					{
						errorMapMaxVal = initialRenderFilm[row][column].predError;
					}
				}
			}

			int extentResolutionDiff = sampleExtent.x - resX;

			int counterBlue = 0;
			int counterGreen = 0;
			int counterRed = 0;

			for (int row = 0; row < resX; row++)
			{
				for (int column = 0; column < resY; column++)
				{
					// Initializing the color ramp
					Float tmpErrorVal = (Float)initialRenderFilm[row][column].predError / errorMapMaxVal * scaleFactorErrorMap;
					tmpErrorVal *= 3;
					tmpErrorVal = std::min((Float)3.0, tmpErrorVal);
					int tmpErrorValFloor = tmpErrorVal;

					Float red = 0.0;
					Float green = 0.0;
					Float blue = 0.0;

					Float floorDiff = (tmpErrorVal - (Float)tmpErrorValFloor);

					if (tmpErrorValFloor == 0)
					{
						counterBlue++;
						blue = 1.0 - floorDiff;
						green = floorDiff;
					}
					else if (tmpErrorValFloor == 1)
					{
						blue = floorDiff;
						counterGreen++;
						green = 1.0 - floorDiff;
					}
					else if (tmpErrorValFloor == 2)
					{
						green = floorDiff;
						counterRed++;
						red = 1.0 - floorDiff;
					}
					else if (tmpErrorValFloor == 3)
					{
						counterRed++;
						red = 1.0;
					}

					//printf("\n\nRGB: [ %f , %f , %f ]\n\n", red, green, blue);
					//printf("\n***********************************************************************************************\n\n");

					arrayToFill[3 * (column * sampleExtent.x + row) + 0 + 3 * extentResolutionDiff] = red;
					arrayToFill[3 * (column * sampleExtent.x + row) + 1 + 3 * extentResolutionDiff] = green;
					arrayToFill[3 * (column * sampleExtent.x + row) + 2 + 3 * extentResolutionDiff] = blue;
				}
			}
		},
		"ErrorEstimationMap_", fileName, sampleBounds, fullResolution, 0.0, 0.0, 1.0);

		/////////////////////////////////////////////////////////////////////////////////////////
		// Visualize coverage mask

		writeVisualizationToImage([&](Float* arrayToFill)
		{
			int coverageMaskMaxVal = 0;
			for (int row = 0; row < sampleExtent.x; row++)
			{
				for (int column = 0; column < sampleExtent.y; column++)
				{
					if (coverageMaskMaxVal < coverageMask[row][column].coverageCounter)
					{
						coverageMaskMaxVal = coverageMask[row][column].coverageCounter;
					}
				}
			}

			for (int row = 0; row < sampleExtent.x; row++)
			{
				for (int column = 0; column < sampleExtent.y; column++)
				{
					arrayToFill[3 * (column * sampleExtent.x + row) + 0] = 1.0 - (Float)coverageMask[row][column].coverageCounter / (Float)coverageMaskMaxVal;
					arrayToFill[3 * (column * sampleExtent.x + row) + 1] = 1.0 - (Float)coverageMask[row][column].coverageCounter / (Float)coverageMaskMaxVal;
					arrayToFill[3 * (column * sampleExtent.x + row) + 2] = 1.0 - (Float)coverageMask[row][column].coverageCounter / (Float)coverageMaskMaxVal;
				}
			}
		},
		"CoverageMask_", fileName, sampleBounds, fullResolution, 1.0, 1.0, 1.0);
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Values for Eigen testing.
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double testM[] = { -34.7622, -23.6197, 40.2716, -14.6841, 42.9386, 8.7045, -24.1935, 7.8525, -16.4643, 10.9867, 2.1650, 30.3364, -44.7323, 6.1200, -24.8194, -32.1234, -39.4371, 17.1808, -17.7528,
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

	double testV[] = { 47.9746,
						32.7870,
						1.7856,
						42.4565,
						46.6997,
						33.9368,
						37.8870,
						37.1566,
						19.6114,
						32.7739,
						8.5593,
						35.3023,
						1.5916,
						13.8461,
						2.3086,
						4.8566,
						41.1729,
						34.7414,
						15.8550 };

	void testMatrixInv() 
	{
		Eigen::MatrixXd m(19, 19);

		for (int i = 0; i < 19; i++)
		{
			for (int j = 0; j < 19; j++)
			{
				m(i, j) = (double)testM[i * 19 + j];
			}
		}
		//std::cout << "\nM: \n" << m << std::endl;
		//std::cout << "Inverse M: \n" << m.inverse() << std::endl;
	}

	void testLUFactorization() 
	{
		Eigen::MatrixXd X(19, 19);

		for (int i = 0; i < 19; i++)
		{
			for (int j = 0; j < 19; j++)
			{
				X(i, j) = (double)testM[i * 19 + j];
			}
		}

		//std::cout << "\nX: \n" << X << std::endl;

		Eigen::VectorXd result;

		Eigen::VectorXd Y(19,1);

		for (int i = 0; i < 19; i++)
		{
			Y(i, 1) = testV[i];
		}

		//std::cout << "\nY: \n" << Y << std::endl;

		Eigen::MatrixXd A = X.transpose()*X;
		Eigen::VectorXd B = X.transpose()*Y;

		//std::cout << "X rows dimension: " << X.rows() << std::endl;
		//std::cout << "X cols dimension: " << X.cols() << std::endl;
		//std::cout << "Y rows dimension: " << Y.rows() << std::endl;
		//std::cout << "Y cols dimension: " << Y.cols() << std::endl;
		// Add error handling here!

		result = Eigen::VectorXd(19, 1);
		result = (X.transpose()*X).inverse()*X.transpose()*Y;
		//std::cout << "\nNormal equation solution:\n" << result << std::endl;
		//std::cout << "\nSolution rows dimension: " << (A.fullPivLu().solve(B)).rows() << std::endl;
		//std::cout << "\nSolution columns dimension: " << (A.fullPivLu().solve(B)).cols() << std::endl;
		result = A.fullPivLu().solve(B);
		//std::cout << "\nLU decomposition solution:\n" << result << std::endl;
	}

	// Duplicates for testing only! Delete later!
	void TESTupdatePredictionErrorEstimate(LinearModel &linModel, const LinearModel previousLinModel, const std::vector<std::vector<rawPixelData>>& rawPixelData, Eigen::MatrixXd Xc, Eigen::MatrixXd Yc)
	{
		std::vector<Float> linModelErrors;
		Eigen::MatrixXd XcT;

		Float linModelError;
		Float newLinModelError;

		int windowSize = linModel.windowSize;

		assert((windowSize % 2) != 0, "Linear model window size must be 1,3,5,...(2r+1)");
		//  If first prediction error.
		if (windowSize == 1)
		{
			linModelError = linModel.linModelCoeffs(1, 1)*Xc(1, 1) + linModel.linModelCoeffs(1, 2)*Xc(1, 2);
		}
		else
		//  If second prediction error.
		if (windowSize == 3)
		{
			//XcT = Xc.transpose();
			//Eigen::MatrixXd Pc = (Xc.transpose()*Xc).inverse();
			//for (int i = 0; i < 9; i++)
			//{
			//	newLinModelError = ((linModel.linModelCoeffs.row(i)*Xc.row(i).transpose())(1, 1) - Yc(i)) / (1 - (XcT.row(i).transpose()*Pc*XcT.row(i))(1, 1));
			//	newLinModelError *= newLinModelError;
			//}
			//linModelError = newLinModelError;
			linModelError = 0;
			XcT = Xc.transpose();
			if ((Xc.transpose()*Xc).determinant() != 0.0)
			{
				Eigen::MatrixXd Pc = (Xc.transpose()*Xc).inverse();
				for (int i = 0; i < 9; i++)
				{
					// DEBUG!
					Float nominator = ((linModel.linModelCoeffs.transpose() * Xc.row(i).transpose())(0, 0) - Yc(i));
					Float denominator = (1.0 - (Xc.row(i) * Pc * Xc.row(i).transpose())(0, 0))+std::numeric_limits<Float>::min();

					//std::cout << "\n\n LinModelCoeffs result (matrix): \n" << (linModel.linModelCoeffs.transpose()) << std::endl << std::endl;

					//std::cout << "\n\n Error XcT result (matrix): \n" << XcT << std::endl << std::endl;

					//std::cout << "\n\n Error Pc result (matrix): \n" << Pc << std::endl << std::endl;

					//if (std::numeric_limits<Float>::epsilon() == denominator || denominator == 0.0)
					//{
						//printf("\n\nNominator / Denominator (prediction error, window size 3): [ %f / %f ]\n\n", nominator, denominator);
					//}

					Eigen::MatrixXd denomnatorMatrix = (Xc.row(i) * Pc * Xc.row(i).transpose());
					//std::cout << "\n\nZT: \n" << Xc.row(i) << std::endl;
					//std::cout << "\nZ:   \n" << Xc.row(i).transpose() << std::endl << std::endl;
					//std::cout << "\n\n Error denominator result (matrix) (Xc.row(i).transpose() * Pc * Xc.row(i)): \n" << denomnatorMatrix(0,0) << std::endl << std::endl;

					//newLinModelError = ((linModel.linModelCoeffs.transpose() * Xc.row(i))(0, 0) - Yc(i)) / (1.0 - denomnatorMatrix(0,0));//(Xc.row(i) * Pc * Xc.row(i).transpose())(1, 1));
					newLinModelError = nominator / (1.0 - denomnatorMatrix(0, 0));
					newLinModelError *= newLinModelError;
					linModelError += newLinModelError;
					linModel.nominatorPredError = linModelError;
				}
			}
			else { linModelError = 0; }
		}
		else
			// If remaining nth prediction errors
			if (windowSize > 3)
			{
				XcT = Xc.transpose();
				linModelError = previousLinModel.nominatorPredError;
				int r = (windowSize / 2);

				for (int i = 0; i < (windowSize * windowSize - 1); i++)
				{
					if (i < windowSize || i % windowSize == 0 || i % windowSize == windowSize - 1 || i >= (windowSize * windowSize) - windowSize)
					{
						//Float tmpLinModelError = (linModel.linModelCoeffs.row(i)*Xc.row(i).transpose())(1, 1) - Yc(i);
						//newLinModelError += (tmpLinModelError*tmpLinModelError);
							
						// DEBUG!
						//Float nominator = (linModel.linModelCoeffs.row(i)*Xc.row(i).transpose())(0, 0) - Yc(i);
						//Float denominator = (Float)((2 * outerRingLength + 1) * (2 * outerRingLength + 1));
							
						//Eigen::MatrixXd tmp = (linModel.linModelCoeffs.row(i)*Xc.row(i).transpose());
						//std::cout << "\n\n Error update result (matrix): \n" << tmp << std::endl << std::endl;
						Float tmpLinModelError = (previousLinModel.linModelCoeffs.transpose()*Xc.row(i).transpose())(0, 0) - Yc(i);
						newLinModelError += (tmpLinModelError*tmpLinModelError);
					}
					else
						continue;
				}
				linModelError += newLinModelError;
				linModel.nominatorPredError = linModelError;
				linModelError /= (Float)((2 * r + 1) * (2 * r + 1));
			}
		linModel.predError = linModelError;

		//printf("\n//////////////////////////////////////////////////////////////////////////////\n");
		//printf("Linear model window size: %d\n", windowSize);
		//printf("Linear model prediction error: %f\n", linModel.predError);
		//printf("//////////////////////////////////////////////////////////////////////////////\n");
	}

	LinearModel TESTcomputeLinearModelAndPredictionError(const LinearModel previousLinModel, int adaptiveWindowSize, const std::vector<std::vector<rawPixelData>>& rawPixelData, Point2i centerPixel)
	{
		LinearModel result;

		result.center = centerPixel;
		result.windowSize = adaptiveWindowSize;

		Eigen::MatrixXd X = constructXc(adaptiveWindowSize);
		Eigen::VectorXd Y = constructYc(adaptiveWindowSize, rawPixelData, centerPixel);

		Eigen::MatrixXd A = X.transpose()*X;
		Eigen::VectorXd B = X.transpose()*Y;

		assert(X.transpose().cols() == Y.rows(), "Matrix-vector multiplication dimension missmatch (Matirix columns not equal to vector rows).");

		result.linModelCoeffs = Eigen::VectorXd(adaptiveWindowSize, Y.cols());
		result.linModelCoeffs = A.fullPivLu().solve(B);

		// Update prediction error of the linear model here.
		TESTupdatePredictionErrorEstimate(result, previousLinModel, rawPixelData, X, Y);

		return result;
	}

	void LPSamplingPlanner::predictionErrorEstimateTest()
	{
		// Test, all valeus of 21x21 matrix are equal. Expected result: Zero, for all window sizes up to fixed window size.
		//std::vector<std::vector<rawPixelData>> testAllEqualPixelData = std::vector<std::vector<rawPixelData>>(21, std::vector<rawPixelData>(21, rawPixelData()));

		//Float fillVal = 1.0;
		//Float fillVal1 = 1.0;
		//Float fillVal2 = 0.0;
		//Float fillVal3 = 255.0;


		//int cut = 7;
		//for (int i = 0; i < 21; i++)
		//{
		//	for (int j = 0; j < 21; j++)
		//	{
		//		testAllEqualPixelData[i][j].xyz[0] = i * 21 + j;
		//		testAllEqualPixelData[i][j].xyz[1] = i * 21 + j;
		//		testAllEqualPixelData[i][j].xyz[2] = i * 21 + j;

		//		testAllEqualPixelData[i][j].rgb[0] = i * 21 + j;
		//		testAllEqualPixelData[i][j].rgb[1] = i * 21 + j;
		//		testAllEqualPixelData[i][j].rgb[2] = i * 21 + j;
		//	}
		//}

		int fixedWindowSize = 5;

		Float fillVal = 1.0;

		std::vector<std::vector<rawPixelData>> testAllEqualPixelData = std::vector<std::vector<rawPixelData>>(fixedWindowSize, std::vector<rawPixelData>(fixedWindowSize, rawPixelData()));

		int cut = 2;

		Float fillVal2 = 2;

		for (int i = 0; i < fixedWindowSize; i++)
		{
			for (int j = 0; j < fixedWindowSize; j++)
			{
				if (i>=cut)
				{
					//fillVal = i * fixedWindowSize + j;
					testAllEqualPixelData[i][j].xyz[0] = fillVal2;
					testAllEqualPixelData[i][j].xyz[1] = fillVal2;
					testAllEqualPixelData[i][j].xyz[2] = fillVal2;

					testAllEqualPixelData[i][j].rgb[0] = fillVal2;
					testAllEqualPixelData[i][j].rgb[1] = fillVal2;
					testAllEqualPixelData[i][j].rgb[2] = fillVal2;
				}
				else 
				{
					//fillVal = i * fixedWindowSize + j;
					testAllEqualPixelData[i][j].xyz[0] = fillVal;
					testAllEqualPixelData[i][j].xyz[1] = fillVal;
					testAllEqualPixelData[i][j].xyz[2] = fillVal;

					testAllEqualPixelData[i][j].rgb[0] = fillVal;
					testAllEqualPixelData[i][j].rgb[1] = fillVal;
					testAllEqualPixelData[i][j].rgb[2] = fillVal;
				}

			}
		}

		std::vector<LinearModel> linModels;

		Point2i centerPixel = Point2i(fixedWindowSize/2, fixedWindowSize/2);

		for (int i = 1; i < fixedWindowSize+1; i+=2)
		{
			//Float sumAllElementsOfXc = 0;
			//Float sumAllElementsOfYc = 0;
			//Eigen::MatrixXd Xc = constructXc(i, testAllEqualPixelData, centerPixel);
			//Eigen::VectorXd Yc = constructYc(i, testAllEqualPixelData, centerPixel);
			
			//for (int j = 0; j < Xc.cols()*Xc.rows(); j++)
			//{
			//	sumAllElementsOfXc += Xc(j); 
			//}

			//for (int j = 0; j < Yc.cols()*Yc.rows(); j++)
			//{
			//	sumAllElementsOfYc += Yc(j);
			//}

			//printf("\n\nSum of all Xc elements: %f\n",sumAllElementsOfXc);
			//printf("Sum of all Yc elements: %f\n\n", sumAllElementsOfYc);
			if (!linModels.empty())
			{
				linModels.push_back(computeLinearModelAndPredictionError(linModels[linModels.size() - 1], i, testAllEqualPixelData, centerPixel));
			}
			else
			{
				linModels.push_back(computeLinearModelAndPredictionError(LinearModel(), i, testAllEqualPixelData, centerPixel));
			}
			
		}

		for (int i = 0; i < linModels.size(); i++)
		{
			printf("\n\nLinear model prediction error for window size [ %d ]: [ %f ]\n\n", linModels[i].windowSize, linModels[i].predError);
			std::cout << "\nLinear model : \n" << linModels[i].linModelCoeffs << "\n\n";
		}

		// Test, all valeus of 19x19 matrix are equal. Expected result: Some solution, already tested for solveability.

		//std::vector<std::vector<rawPixelData>> testRandomPixelData = std::vector<std::vector<rawPixelData>>(19, std::vector<rawPixelData>(19, rawPixelData()));

		//for (int i = 0; i < 19; i++)
		//{
		//	for (int j = 0; j < 19; j++)
		//	{
		//		testRandomPixelData[i][j].rgb[0] = testM[i * 19 + j];
		//		testRandomPixelData[i][j].rgb[1] = testM[i * 19 + j];
		//		testRandomPixelData[i][j].rgb[2] = testM[i * 19 + j];
		//	}
		//}

		//linModels.clear();

		//for (int i = 1; i < 18; i += 2)
		//{
		//	Float sumAllElementsOfXc = 0;
		//	Float sumAllElementsOfYc = 0;
		//	Eigen::MatrixXd Xc = constructXc(i, testRandomPixelData, Point2i(10, 10));
		//	Eigen::VectorXd Yc = constructYc(i, testRandomPixelData, Point2i(10, 10));

		//	for (int j = 0; j < Xc.cols()*Xc.rows(); j++)
		//	{
		//		sumAllElementsOfXc += Xc(j);
		//	}

		//	for (int j = 0; j < Yc.cols()*Yc.rows(); j++)
		//	{
		//		sumAllElementsOfYc += Yc(j);
		//	}

		//	printf("\n\nSum of all Xc elements: %f\n", sumAllElementsOfXc);
		//	printf("Sum of all Yc elements: %f\n\n", sumAllElementsOfYc);

		//	linModels.push_back(TESTcomputeLinearModelAndPredictionError(i, testRandomPixelData, Point2i(10, 10)));
		//}

		//for (int i = 0; i < linModels.size(); i++)
		//{
		//	printf("\n\nLinear model prediction error for window size [ %d ]: [ %f ]\n\n", linModels[i].windowSize, linModels[i].predError);
		//}
	}
}