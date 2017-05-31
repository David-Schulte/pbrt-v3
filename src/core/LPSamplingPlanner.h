#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_LPSAMPLINGPLANNER_H
#define PBRT_CORE_LPSAMPLINGPLANNER_H

#include "SamplingPlanner.h"

namespace pbrt
{

	class LPSamplingPlanner : public SamplingPlanner
	{
	public:
		LPSamplingPlanner() {}
		~LPSamplingPlanner() {}

		virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount=0) override;

	protected:
		virtual void CreateSamplingPlan(int samplesPerPixel, Film * film) override;
	};

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H