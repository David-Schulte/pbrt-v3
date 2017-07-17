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
        int filterRadius = 1;
        int patchRadius = 1;
        Float cancellationFactor = 1;
        Float dampingFactor = 1;

        NLMeansFilter();
        ~NLMeansFilter();

        Float PixelWeightSum(const Buffer &weightSourceBuffer, const Point2i &pixel, const Buffer &weightSourceVarianceBuffer);

        Buffer Filter(Film * film, int weightSourceBuffer, int filterBuffer);

    protected:

        Buffer EstimateVariance(Film * film, int weightSourceBuffer, int filterBuffer);

        Buffer DirectFilter(const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer);
        std::vector<Float> FilterPixel(const Point2i &pixel, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer);
        Float PatchWeight(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer);
        Float PatchDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer);
        std::vector<Float> PixelDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer);

        Bounds2i SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H