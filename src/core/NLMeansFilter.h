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
        using Buffer = std::vector<std::vector<std::vector<Float>>>;

    public:
        NLMeansFilter();
        ~NLMeansFilter();

        std::vector<std::vector<std::vector<Float>>> Filter(const Buffer &weightSourceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius);

    protected:
        std::vector<Float> FilterPixel(const Point2i &pixel, const Buffer &weightSourceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius);
        Float PatchWeight(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, int radius, Float dampingFactor);
        Float PatchDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, int radius);
        Bounds2i SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H