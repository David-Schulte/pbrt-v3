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
		AdaptiveGrid(int fixedWindowSize = 19) : fixedWindowSize(fixedWindowSize), granularity(fixedWindowSize / 2) {}
		void refineGrid() { granularity /= 2; }

		float granularity;
		int64_t fixedWindowSize;
	};
	struct vector_bool
	{
		vector_bool() : value(false) {};
		bool value;
	};

	class LPSamplingPlanner : public SamplingPlanner
	{
	public:
		LPSamplingPlanner(int64_t resolutionX, int64_t resolutionY) : grid(AdaptiveGrid()), coverageMask(std::vector<std::vector<vector_bool>>(resolutionX, std::vector<vector_bool>(resolutionY, vector_bool()))){ }
		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;
		

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;

	private:
		AdaptiveGrid grid;
		std::vector<std::vector<vector_bool>> coverageMask;
	
	};



	

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H