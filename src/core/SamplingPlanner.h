#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_SAMPLINGPLANNER_H
#define PBRT_CORE_SAMPLINGPLANNER_H

#include "pbrt.h"
#include "geometry.h"

namespace pbrt
{

    class SamplingPlanner
    {
    public:
        SamplingPlanner();
        ~SamplingPlanner();

        void InitializeSamplingPlan(int samplesPerPixel, Film *film);
        virtual void UpdateSamplingPlan(Film *film) = 0;
        bool StartNextIteration();

        int PlannedSamples(Point2i &pixel) { return sampleMap[pixel.x+2][pixel.y+2]; }

        int currentAdaptiveIteration;
        int plannedAdaptiveIterations;
        int maxPixelSamplesPerIteration; //For each iteration

    protected:
        std::vector<std::vector<int>> sampleMap;
        int sampleBudgetPerPixel;

        virtual void CreateSamplingPlan(int samplesPerPixel, Film *film) = 0;
        void CreateSampleMap(Film *film);
    };

}

#endif //PBRT_CORE_SAMPLINGPLANNER_H