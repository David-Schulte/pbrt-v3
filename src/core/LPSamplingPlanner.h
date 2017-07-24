#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_LPSAMPLINGPLANNER_H
#define PBRT_CORE_LPSAMPLINGPLANNER_H

#include "SamplingPlanner.h"
#include <Eigen/Dense>

namespace pbrt
{
	struct AdaptiveGrid
	{
		AdaptiveGrid(int fixedWindowSize = 19) : fixedWindowSize(fixedWindowSize), granularity(fixedWindowSize) {}
		void refineGrid() { granularity /= 2; margin.x /= 2; margin.y /= 2; }

		Point2i margin;
		int granularity;
		int64_t fixedWindowSize;
	};

	struct vector_bool
	{
		vector_bool() : value(false) {};
		bool value;
		int coverageCounter = 0;
	};

	struct rawPixelData
	{
		rawPixelData(Float* _xyz) { xyz[0] = _xyz[0]; xyz[1] = _xyz[1]; xyz[2] = _xyz[2]; }
		rawPixelData() { xyz[0] = 0; xyz[1] = 0; xyz[2] = 0; }
		Float xyz[3] = { 0,0,0 };
		Float rgb[3] = { 0,0,0 };

		// For visualization only!
		Float predError = 0.0;
	};

	struct LinearModel
	{
		Point2i center;
		int windowSize;
		Eigen::VectorXd linModelCoeffs;
		double nominatorPredError = 0.0;
		double predError = 0.0;
		void LinearModel::print() 
		{
			printf("\nCenter Pixel: %d, %d\n", center.x, center.y);
			printf("Window Size: %d, %d\n", windowSize, windowSize);
			printf("linModelCoeffs: %.12f, %.12f\n", linModelCoeffs(0,0), linModelCoeffs(1,0));
			printf("nominatorPredError: %.12f\n", nominatorPredError);
			printf("predError: %.12f\n", predError);
		}
	};

	class LPSamplingPlanner : public SamplingPlanner
	{
	public:
		LPSamplingPlanner(int64_t resolutionX, int64_t resolutionY) :
			grid(AdaptiveGrid()),
			initialRenderFilmReady(false),
			finalRender(false),
			numberCoveredPixels(0){ }

		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;
		virtual bool StartNextIteration();

		bool initialRenderFilmReady;

		std::vector<std::vector<rawPixelData>> getInitialRenderFilm() { return initialRenderFilm; }
		std::vector<std::vector<vector_bool>> getCoverageMask() { return coverageMask; }

		// For test only!
		void predictionErrorEstimateTest();

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;
		

	private:

		AdaptiveGrid grid;
		std::vector<std::vector<vector_bool>> coverageMask;
		std::vector<std::vector<rawPixelData>> initialRenderFilm;
		std::vector<std::vector<int64_t>> temp_plannedSampleMap;
		bool finalRender;
		int64_t numberCoveredPixels;
		int32_t featureDim = 2;
		int32_t minSamplesSend = 0;//DEBUG
		int32_t maxSamplesSend = 0;//DEBUG
		double maxError = 0;
		double minError = 0;
		int32_t maxLevelOfAdaptation = 3; 
		int32_t currentLevelOfAdaptation = 0;

		LinearModel computeLinearModelAndPredictionError(const LinearModel previousLinModel, int adaptiveWindowSize, const std::vector<std::vector<rawPixelData>>& rawPixelData, Point2i centerPixel);
		void updatePredictionErrorEstimate(LinearModel &linModel, const LinearModel previousLinModel, const std::vector<std::vector<rawPixelData>>& rawPixelData, Eigen::MatrixXd Xc, Eigen::MatrixXd Yc);
		
		int findMinErrorLinModelIdx(std::vector<LinearModel> linModels);
		int64_t getPlannedSampleNumber(LinearModel minErrorLinModel, const pbrt::Point2i pixel, int32_t additionalSampleStep);
		virtual void copyInitialRenderFilm(Film* film);
		virtual Point2i computeMargin(Film* film);
		void initForAdaptiveIterations(Film * film);
		bool allPixelsCoveredByAtLeastOneLinearModel(Film* film);
		void averagePlannedSampleNumber(Film* film);
		bool isPixelPartOfImage(pbrt::Point2i pixel);
		bool windowReachesOverBorder(pbrt::Point2i centerPixel, int32_t windowSize);
		std::vector<LinearModel> computeAllLinearModels(int32_t centerPixelX, int32_t centerPixelY);
	};
}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H