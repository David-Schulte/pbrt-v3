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
		AdaptiveGrid(int fixedWindowSize = 191) : fixedWindowSize(fixedWindowSize), granularity(fixedWindowSize) {}
		void refineGrid() { granularity /= 2; }

		float granularity;
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
		LPSamplingPlanner(int64_t resolutionX, int64_t resolutionY) : grid(AdaptiveGrid()), coverageMask(std::vector<std::vector<vector_bool>>(resolutionX, std::vector<vector_bool>(resolutionY, vector_bool()))), initialRenderFilmReady(false){ }
		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;
		

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;
		virtual void copyInitialRenderFilm(Film* film);
		

	private:
		AdaptiveGrid grid;
		std::vector<std::vector<vector_bool>> coverageMask;
		std::vector<std::vector<rawPixelData>> initialRenderFilm;
		bool initialRenderFilmReady;
	};



	

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H