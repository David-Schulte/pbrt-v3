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

        Buffer Filter(Film * film, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor);

    protected:

        Buffer EstimateVariance(Film * film, int weightSourceBuffer, int filterBuffer);

        Buffer DirectFilter(const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor);
        std::vector<Float> FilterPixel(const Point2i &pixel, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor);
        Float PatchWeight(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer, int radius, Float cancellationFactor, Float dampingFactor);
        Float PatchDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer, int radius, Float cancellationFactor, Float dampingFactor);
        std::vector<Float> PixelDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer, Float cancellationFactor, Float dampingFactor);

        Bounds2i SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H