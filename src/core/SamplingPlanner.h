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
        virtual void UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount = 0) = 0; // Second parameter (_adaptiveSamplesCount_) is for debug purposes.
		virtual void UpdateCurrentSampleNumberMap();
        virtual bool StartNextIteration();
		void setInitialRenderSamples(int initialSamplesPerPixel) {initialRenderSamplesPerPixels = initialSamplesPerPixel;}

        const int PlannedSamples(const Point2i &pixel) { return plannedSampleMap[pixel.x + filmExtentResDiff/2][pixel.y + filmExtentResDiff/2]; }
		const int CurrentSampleNumber(const Point2i &pixel) { return currentSampleNumberMap[pixel.x + filmExtentResDiff/2][pixel.y + filmExtentResDiff/2]; }

		void PlannedAdaptiveIterations(int plannedAdaptiveIterations) { this->plannedAdaptiveIterations = plannedAdaptiveIterations; }

        int currentAdaptiveIteration;
        int plannedAdaptiveIterations;
        int maxPixelSamplesPerIteration; //For each iteration
		bool firstIteration = true;

		std::vector<Point2i> getAllCenterPixel() { return allCenterPixel; }

    protected:

        std::vector<std::vector<int64_t>> plannedSampleMap;				// Holds number of samples to be used in the current iteration.
		std::vector<std::vector<int64_t>> currentSampleNumberMap;	// Accumulates _sampleMap_ entries elementwise (+=).
		std::vector<Point2i> allCenterPixel;
        int sampleBudgetPerPixel;
		int initialRenderSamplesPerPixels;
		

        virtual void CreateSamplingPlan(int samplesPerPixel, Film *film) = 0;
        void CreateSampleMap(Film *film);
		int64_t filmExtentResDiff;
    };

}

#endif //PBRT_CORE_SAMPLINGPLANNER_H