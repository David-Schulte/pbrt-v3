#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_LPSAMPLINGPLANNER_H
#define PBRT_CORE_LPSAMPLINGPLANNER_H

#include "SamplingPlanner.h"

namespace pbrt
{
	struct AdaptiveGrid
	{
		AdaptiveGrid(int fixedWindowSize = 19) : fixedWindowSize(fixedWindowSize), granularity(fixedWindowSize) {}
		void refineGrid() { granularity /= 2; }

		int granularity;
		int64_t fixedWindowSize;
	};

	struct vector_bool
	{
		vector_bool() : value(false) {};
		bool value;
	};

	struct rawPixelData
	{
		rawPixelData(Float* _xyz) { xyz[0] = _xyz[0]; xyz[1] = _xyz[1]; xyz[2] = _xyz[2]; }
		rawPixelData() { xyz[0] = 0; xyz[1] = 0; xyz[2] = 0; }
		Float xyz[3];
	};

	class LPSamplingPlanner : public SamplingPlanner
	{
	public:
		LPSamplingPlanner(int64_t resolutionX, int64_t resolutionY) :
			grid(AdaptiveGrid()),
			coverageMask(std::vector<std::vector<vector_bool>>(resolutionX+2, std::vector<vector_bool>(resolutionY+2, vector_bool()))),
			initialRenderFilmReady(false),
			finalRender(false),
			numberCoveredPixels(0){ }

		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;
		virtual bool StartNextIteration();

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;
		virtual void copyInitialRenderFilm(Film* film);
		virtual Point2i computeMargin(Film* film);

	private:
		AdaptiveGrid grid;
		std::vector<std::vector<vector_bool>> coverageMask;
		std::vector<std::vector<rawPixelData>> initialRenderFilm;
		std::vector<std::vector<int64_t>> temp_plannedSampleMap; //will be used to accumulate planned sample number until every pixel is covered
		bool initialRenderFilmReady;
		bool finalRender;
		int64_t numberCoveredPixels; 
	};



	

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H