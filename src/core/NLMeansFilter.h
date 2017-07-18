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

        void SetParameters(int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor);

        Float FilterWeightSum(const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Point2i &pixel);
        Buffer Filter(Film * film, int filterSourceBuffer, int weightSourceBuffer);

    protected:
        //Filter settings
        int filterRadius = 1;
        int patchRadius = 1;
        Float cancellationFactor = 1;
        Float dampingFactor = 1;

        //Internal reserved set of filter settings used to restore the original settings when the filter needs to change temporarily internally
        int reservedFilterRadius = 1;
        int reservedPatchRadius = 1;
        Float reservedCancellationFactor = 1;
        Float reservedDampingFactor = 1;

        void ReserveParameters();
        void RestoreParameters();

        //Building blocks of the NLMeansFilter
        Buffer EstimateVariance(Film * film, int filterSourceBuffer, int weightSourceBuffer);
        std::vector<Float> FilterPixel(const Buffer &filterSourceBuffer, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Point2i &pixel);
        Float PatchWeight(const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2);
        Float PatchDistance(const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2);
        std::vector<Float> PixelDistance(const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2);

        Bounds2i SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H