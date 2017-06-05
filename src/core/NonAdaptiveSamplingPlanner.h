#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H
#define PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H

#include "SamplingPlanner.h"

namespace pbrt
{

    class NonAdaptiveSamplingPlanner : public SamplingPlanner
    {
    public:
        NonAdaptiveSamplingPlanner();
        ~NonAdaptiveSamplingPlanner();

    protected:
        virtual void UpdateSampleMap(Film * film) override;
        virtual void CreateSamplingPlan(Film * film) override;
    };

}

#endif //PBRT_CORE_NONADAPTIVESAMPLINGPLANNER_H