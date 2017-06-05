#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_SAMPLINGPLANNER_H
#define PBRT_CORE_SAMPLINGPLANNER_H

#include "pbrt.h"
#include "geometry.h"
#include <inttypes.h>

namespace pbrt
{

    class SamplingPlanner
    {
    public:
        SamplingPlanner();
        ~SamplingPlanner();

        void Initialize(int samplesPerPixel, Film *film);
        void UpdateSamplingPlan(Film *film);
        bool StartNextIteration();
        int SamplesOfPreviousIterations(const Point2i &pixel);
        int PlannedSamplesForThisIteration(const Point2i &pixel);

        int currentIteration;
        int plannedIterations;
        int maxSamplesPerPixel; //For each iteration

    protected:
        std::vector<std::vector<int>> sampleMap;
        int sampleBudgetPerPixel;
        int64_t totalSampleBudget;
        int filmWidth;
        int filmHeight;
        
        virtual void CreateSamplingPlan(Film *film) = 0;
        virtual void UpdateSampleMap(Film *film) = 0;
        void CreateSampleMap(Film *film);
        void FillMapUniformly(std::vector<std::vector<int>> & map, int value);

    private: 
        std::vector<std::vector<int>> totalDistribution;

        void AddSampleMapToDistribution();
    };

}

#endif //PBRT_CORE_SAMPLINGPLANNER_H