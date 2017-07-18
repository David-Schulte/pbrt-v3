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
        using Buffer = std::vector<std::vector<std::vector<Float>>>;

    public:
        NLMeansSamplingPlanner(int initialBudgetTarget, int iterationBudgetTarget, int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor);
        ~NLMeansSamplingPlanner();

    protected:
        std::shared_ptr<NLMeansFilter> filter;
        int initialBudget = 10; //How many samples per pixel are aimed for in the first iteration
        int iterationBudget = 10; //How many samples per pixel are aimed for in each subsequent iteration
        std::vector<int> iterationBudgets; //Average samples per pixel, for each iteration
        std::vector<int> maxSampleBudgets; //Maximum samples per pixel, for each iteration

        void PlanIterations();
        void PlanMaximalSamplesPerPixel();
        void DualBufferFiltering(Film * film);
        Buffer EstimateError(Film * film);
        void FillMapErrorProportional(const Buffer &error);
        virtual void UpdateSampleMap(Film * film) override;
        virtual void CreateSamplingPlan(Film * film) override;
    };

}

#endif //PBRT_CORE_NLMEANSSAMPLINGPLANNER_H