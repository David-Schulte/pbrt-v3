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
        virtual void UpdateSamplingPlan(Film *film) = 0;
        bool StartNextIteration();

        int PlannedSamples(Point2i &pixel) { return sampleMap[pixel.x][pixel.y]; }

        int currentIteration;
        int plannedIterations;
        int maxPixelSamplesPerIteration; //For each iteration

    protected:
        std::vector<std::vector<int>> sampleMap;
        int sampleBudgetPerPixel;
        int64_t totalSampleBudget;
        int filmWidth;
        int filmHeight;

        virtual void CreateSamplingPlan(Film *film) = 0;
        void CreateSampleMap(Film *film);
        void FillMapUniformly(int samplesPerPixel);
    };

}

#endif //PBRT_CORE_SAMPLINGPLANNER_H