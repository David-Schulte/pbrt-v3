#include "NLMeansFilter.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{
    NLMeansFilter::NLMeansFilter() {}
    NLMeansFilter::~NLMeansFilter() {}

    std::vector<std::vector<std::vector<float>>> NLMeansFilter::Filter(Film * film, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius)
    {
        std::vector<std::vector<std::vector<float>>> filteredColors(film->croppedPixelBounds.pMax.x - film->croppedPixelBounds.pMin.x,                                 //size in 1st dimension
                                                                    std::vector<std::vector<float>>(film->croppedPixelBounds.pMax.y - film->croppedPixelBounds.pMin.y, //size in 2nd dimension
                                                                    std::vector<float>(3)));                                                                           //3 color values for each pixel
        
        for (Point2i pixel : film->croppedPixelBounds)
        {
            filteredColors[pixel.x][pixel.y] = FilterPixel(film, pixel, weightSourceBuffer, filterBuffer, filterRadius, patchRadius);
        }

        return filteredColors;
    }

    std::vector<float> NLMeansFilter::FilterPixel(Film * film, Point2i pixel, int weightSourceBuffer, int filterBuffer, int filterRadius, int patchRadius)
    {
        std::vector<float> filteredColor(3);
        Bounds2i bounds = SharedBounds(film, std::vector<Point2i>{pixel}, filterRadius);

        float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offsetPixel = pixel + Point2i(x,y);
                float weight = PatchWeight(film, weightSourceBuffer, pixel, offsetPixel, patchRadius, 0.2);

                Pixel& offsetColor = film->GetPixel(filterBuffer, offsetPixel);
                for (int i = 0; i < 3; i++) filteredColor[i] += weight * offsetColor.xyz[i] / offsetColor.filterWeightSum;

                sumOfWeights += weight;
            }
        }

        for (int i = 0; i < 3; i++) filteredColor[i] /= sumOfWeights;
        return filteredColor;
    }

    float NLMeansFilter::PatchWeight(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius, float dampingFactor)
    {
        //Does not factor in any variance currently
        float distance = PatchDistance(film, buffer, pixel1, pixel2, radius);
        float weight = std::exp(- distance / dampingFactor);
        return weight;
    }

    float NLMeansFilter::PatchDistance(Film * film, int buffer, Point2i pixel1, Point2i pixel2, int radius)
    {
        Bounds2i bounds = SharedBounds(film, std::vector<Point2i>{pixel1, pixel2}, radius);

        float sumOfSqrDifferences;
        int pixelCounter = 0;

        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                Pixel& current1 = film->GetPixel(buffer, pixel1 + offset);
                Pixel& current2 = film->GetPixel(buffer, pixel2 + offset);

                float sqrColorDifference[3];
                for (int i = 0; i < 3; i++)
                {
                    sqrColorDifference[i] = current1.xyz[i] / current1.filterWeightSum - current2.xyz[i] / current2.filterWeightSum;
                    sqrColorDifference[i] = std::pow(sqrColorDifference[i], 2);

                    sumOfSqrDifferences += sqrColorDifference[i];
                }

                pixelCounter++;
            }
        }

        float averageSqrDifference = sumOfSqrDifferences / (pixelCounter * 3);
        return averageSqrDifference;
    }

    Bounds2i NLMeansFilter::SharedBounds(Film * film, std::vector<Point2i> pixels, int radius)
    {
        Bounds2i pixelExtents(pixels[0], pixels[0]);
        for (int i = 0; i < pixels.size(); i++)
        {
            pixelExtents.pMin.x = std::min(pixels[i].x, pixelExtents.pMin.x);
            pixelExtents.pMin.y = std::min(pixels[i].y, pixelExtents.pMin.y);
            pixelExtents.pMax.x = std::max(pixels[i].x, pixelExtents.pMax.x);
            pixelExtents.pMax.y = std::max(pixels[i].y, pixelExtents.pMax.y);
        }
        
        Bounds2i filmBounds = film->croppedPixelBounds;
        Bounds2i sharedBounds;
        sharedBounds.pMin.x = -std::min(pixelExtents.pMin.x - filmBounds.pMin.x, radius);
        sharedBounds.pMin.y = -std::min(pixelExtents.pMin.y - filmBounds.pMin.y, radius);
        sharedBounds.pMax.x = std::min(filmBounds.pMax.x - 1 - pixelExtents.pMax.x, radius); //Film bounds exclude the upper bound value, hence the - 1
        sharedBounds.pMax.y = std::min(filmBounds.pMax.y - 1 - pixelExtents.pMax.y, radius); //Film bounds exclude the upper bound value, hence the - 1

        return sharedBounds;
    }


}
