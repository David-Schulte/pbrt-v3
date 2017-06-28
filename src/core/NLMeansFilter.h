#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_CORE_NLMEANSFILTER_H
#define PBRT_CORE_NLMEANSFILTER_H

#include "sampling.h"

namespace pbrt
{

    class NLMeansFilter
    {
    public:
        NLMeansFilter();
        ~NLMeansFilter();

        std::vector<std::vector<std::vector<float>>> Filter(Film * film, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius);
    protected:
        std::vector<float> FilterPixel(Film * film, Point2i pixel, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius);
        float PatchWeight(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius, float dampingFactor);
        float PatchDistance(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius);
        Bounds2i SharedBounds(Film * film, std::vector<Point2i> pixels, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H