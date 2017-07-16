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
	};

	struct LinearModel
	{
		Point2i center;
		int windowSize;
		Eigen::VectorXd linModelCoeffs;
		double nominatorPredError = 0.0;
		double predError = 0.0;
	};

	class LPSamplingPlanner : public SamplingPlanner
	{
	public:
		LPSamplingPlanner(int64_t resolutionX, int64_t resolutionY) :
			grid(AdaptiveGrid()),
			coverageMask(std::vector<std::vector<vector_bool>>(resolutionX+4, std::vector<vector_bool>(resolutionY+4, vector_bool()))),
			initialRenderFilmReady(false),
			finalRender(false),
			numberCoveredPixels(0){ }

		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;
		virtual bool StartNextIteration();

		bool initialRenderFilmReady;

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;
		

	private:
		const int adaptiveSampleStepSize = 4;

		AdaptiveGrid grid;
		std::vector<std::vector<vector_bool>> coverageMask;
		std::vector<std::vector<rawPixelData>> initialRenderFilm;
		std::vector<std::vector<int64_t>> temp_plannedSampleMap;
		//bool initialRenderFilmReady;
		bool finalRender;
		int64_t numberCoveredPixels;

		LinearModel computeLinearModelAndPredictionError(const LinearModel previousLinModel, int adaptiveWindowSize, const std::vector<std::vector<rawPixelData>>& rawPixelData, Point2i centerPixel);
		void updatePredictionErrorEstimate(LinearModel &linModel, const LinearModel previousLinModel, const std::vector<std::vector<rawPixelData>>& rawPixelData, Eigen::MatrixXd Xc, Eigen::MatrixXd Yc);
		
		int findMinErrorLinModelIdx(std::vector<LinearModel> linModels, Float minErrorThreshold = 0.000000001);
		int64_t getPlannedSampleNumber(LinearModel minErrorLinModel, int64_t additionalSampleStep, Float invMinErrorThresholdFactor = 100000000.0);		//Debug! Currently test implementation for matrix inverse computation
		virtual void copyInitialRenderFilm(Film* film);
		virtual Point2i computeMargin(Film* film);
	};



	

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H