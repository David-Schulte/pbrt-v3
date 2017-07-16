#include "NLMeansFilter.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{
    NLMeansFilter::NLMeansFilter() {}
    NLMeansFilter::~NLMeansFilter() {}

    std::vector<std::vector<std::vector<Float>>> NLMeansFilter::Filter(const Buffer &weightSourceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius)
    {
        Buffer filteredColors(filterBuffer.size(),                                    //size in 1st dimension
                              std::vector<std::vector<Float>>(filterBuffer[0].size(), //size in 2nd dimension
                              std::vector<Float>(3)));                                //3 color values for each pixel
        
        unsigned int startTime = clock();
        int sizeX = filterBuffer.size();
        int sizeY = filterBuffer[0].size();

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredColors[x][y] = FilterPixel(Point2i(x, y), weightSourceBuffer, filterBuffer, filterRadius, patchRadius);

        Float elapsedTime = (Float)(clock() - startTime) / 1000;
        elapsedTime = std::round(elapsedTime * 10) / 10; //Round to one decimal digit
        LOG(INFO) << "Filtering complete, took: " << elapsedTime << " seconds";
    
        return filteredColors;
    }

    std::vector<Float> NLMeansFilter::FilterPixel(const Point2i &pixel, const Buffer &weightSourceBuffer, const Buffer &filterBuffer, int filterRadius, int patchRadius)
    {
        std::vector<Float> filteredColor(3);
        Bounds2i bounds = SharedBounds(weightSourceBuffer, std::vector<Point2i>{pixel}, filterRadius);

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offsetPixel = pixel + Point2i(x,y);
                Float weight = PatchWeight(weightSourceBuffer, pixel, offsetPixel, patchRadius, 0.2);

                std::vector<Float> offsetColor = filterBuffer[offsetPixel.x][offsetPixel.y];
                for (int i = 0; i < 3; i++) filteredColor[i] += weight * offsetColor[i];

                sumOfWeights += weight;
            }
        }

        for (int i = 0; i < 3; i++) filteredColor[i] /= sumOfWeights;
        return filteredColor;
    }

    Float NLMeansFilter::PatchWeight(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, int radius, Float dampingFactor)
    {
        //Does not factor in any variance currently
        Float distance = PatchDistance(buffer, pixel1, pixel2, radius);
        Float weight = std::exp(-distance / dampingFactor);
        return weight;
    }

    Float NLMeansFilter::PatchDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, int radius)
    {
        Bounds2i bounds = SharedBounds(buffer, std::vector<Point2i>{pixel1, pixel2}, radius);

        Float sumOfSqrDifferences;
        int pixelCounter = 0;

        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                std::vector<Float> color1 = buffer[pixel1.x + offset.x][pixel1.y + offset.y];
                std::vector<Float> color2 = buffer[pixel2.x + offset.x][pixel2.y + offset.y];

                Float sqrColorDifference[3];
                for (int i = 0; i < 3; i++)
                {
                    sqrColorDifference[i] = color1[i] - color2[i];
                    sqrColorDifference[i] = sqrColorDifference[i] * sqrColorDifference[i];

                    sumOfSqrDifferences += sqrColorDifference[i];
                }

                pixelCounter++;
            }
        }

        Float averageSqrDifference = sumOfSqrDifferences / (pixelCounter * 3);
        return averageSqrDifference;
    }

    Bounds2i NLMeansFilter::SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius)
    {
        Bounds2i pixelExtents(pixels[0], pixels[0]);
        for (int i = 0; i < pixels.size(); i++)
        {
            pixelExtents.pMin.x = std::min(pixels[i].x, pixelExtents.pMin.x);
            pixelExtents.pMin.y = std::min(pixels[i].y, pixelExtents.pMin.y);
            pixelExtents.pMax.x = std::max(pixels[i].x, pixelExtents.pMax.x);
            pixelExtents.pMax.y = std::max(pixels[i].y, pixelExtents.pMax.y);
        }
        
        Bounds2i sharedBounds;
        sharedBounds.pMin.x = -std::min(pixelExtents.pMin.x - 0, radius);
        sharedBounds.pMin.y = -std::min(pixelExtents.pMin.y - 0, radius);
        sharedBounds.pMax.x = std::min((int)bounds.size()    - 1 - pixelExtents.pMax.x, radius);
        sharedBounds.pMax.y = std::min((int)bounds[0].size() - 1 - pixelExtents.pMax.y, radius);

        return sharedBounds;
    }


}
