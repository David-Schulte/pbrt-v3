#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_NLMEANSSAMPLINGPLANNER_H
#define PBRT_CORE_NLMEANSSAMPLINGPLANNER_H

#include "SamplingPlanner.h"
#include "NLMeansFilter.h"

namespace pbrt
{

    class NLMeansSamplingPlanner : public SamplingPlanner
    {
    public:
        NLMeansSamplingPlanner();
        ~NLMeansSamplingPlanner();

    protected:
        std::shared_ptr<NLMeansFilter> filter;
        const int initialBudgetTarget = 10; //How many samples per pixel are aimed for in the first iteration
        const int iterationBudgetTarget = 5; //How many samples per pixel are aimed for in each subsequent iteration
        std::vector<int> iterationBudgets; //Average samples per pixel, for each iteration

        void PlanIterations();
        void PlanMaximalSamplesPerPixel();
        void DualBufferFiltering(Film * film);
        void EstimateError(Film * film);
        virtual void UpdateSampleMap(Film * film) override;
        virtual void CreateSamplingPlan(Film * film) override;
    };

}

#endif //PBRT_CORE_NLMEANSSAMPLINGPLANNER_H