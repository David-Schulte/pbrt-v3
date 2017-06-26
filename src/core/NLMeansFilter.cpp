#include "NLMeansFilter.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{
    NLMeansFilter::NLMeansFilter() {}
    NLMeansFilter::~NLMeansFilter() {}

    float NLMeansFilter::PatchDistance(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius)
    {
        int xBegin = -radius;
        int yBegin = -radius;
        int xEnd = radius;
        int yEnd = radius;

        if (std::min(pixel1.x, pixel2.x) - radius < 0) xBegin += radius - std::min(pixel1.x, pixel2.x);
        if (std::min(pixel1.y, pixel2.y) - radius < 0) yBegin += radius - std::min(pixel1.y, pixel2.y);
        if (std::max(pixel1.x, pixel2.x) + radius > film->croppedPixelBounds.pMax.x) xEnd -= std::max(pixel1.x, pixel2.x) + radius - film->croppedPixelBounds.pMax.x;
        if (std::max(pixel1.y, pixel2.y) + radius > film->croppedPixelBounds.pMax.y) yEnd -= std::max(pixel1.y, pixel2.y) + radius - film->croppedPixelBounds.pMax.y;

        float sumOfSqrDifferences;
        int pixelCounter = 0;

        for (int x = xBegin; x <= xEnd; x++)
        {
            for (int y = yBegin; y <= yEnd; y++)
            {
                Point2i offset(x, y);
                Pixel& current1 = film->GetPixel(buffer, pixel1 + offset);
                Pixel& current2 = film->GetPixel(buffer, pixel2 + offset);

                Float sqrColorDifference[3];
                for (int i = 0; i < 3; i++)
                {
                    sqrColorDifference[i] = current1.xyz[i] - current2.xyz[i];
                    sqrColorDifference[i] = sqrColorDifference[i] / (current1.filterWeightSum + current2.filterWeightSum);
                    sqrColorDifference[i] = std::pow(sqrColorDifference[i], 2);

                    sumOfSqrDifferences += sqrColorDifference[i];
                }

                pixelCounter++;
            }
        }

        float averageSqrDifference = sumOfSqrDifferences / pixelCounter * 3;
        return averageSqrDifference;
    }


}
