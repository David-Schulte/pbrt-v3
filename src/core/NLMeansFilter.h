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

    protected:
        float PatchDistance(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius);
    };

}

#endif //PBRT_CORE_NLMEANSFILTER_H