#include "NLMeansFilter.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{
    NLMeansFilter::NLMeansFilter() {}
    NLMeansFilter::~NLMeansFilter() {}

    Float NLMeansFilter::PixelWeightSum(const Buffer &weightSourceBuffer, const Point2i &pixel, const Buffer &weightSourceVarianceBuffer)
    {
        Bounds2i bounds = SharedBounds(weightSourceBuffer, std::vector<Point2i>{pixel}, filterRadius);

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offsetPixel = pixel + Point2i(x, y);
                Float weight = PatchWeight(weightSourceBuffer, pixel, offsetPixel, weightSourceVarianceBuffer);
                sumOfWeights += weight;
            }
        }

        return sumOfWeights;
    }

    NLMeansFilter::Buffer NLMeansFilter::Filter(Film * film, int weightSourceBuffer, int filterBuffer)
    {
        Buffer weightSource = film->BufferMean(weightSourceBuffer);
        Buffer filter = film->BufferMean(filterBuffer);

        Buffer filteredColors(filter.size(),                                    //size in 1st dimension
                              std::vector<std::vector<Float>>(filter[0].size(), //size in 2nd dimension
                              std::vector<Float>(3)));                          //3 color values for each pixel
        
        unsigned int startTime = clock();
        int sizeX = filter.size();
        int sizeY = filter[0].size();

        Buffer varianceEstimation = EstimateVariance(film, weightSourceBuffer, filterBuffer);

        //Debug image
        film->SetBuffers(film->amountOfBuffers + 1);
        film->WriteToBuffer(varianceEstimation, film->amountOfBuffers - 1, 1);
        film->WriteBufferImage("varianceEstimation.exr", film->amountOfBuffers - 1);
        film->SetBuffers(film->amountOfBuffers - 1);

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredColors[x][y] = FilterPixel(Point2i(x, y), weightSource, varianceEstimation, filter);

        Float elapsedTime = (Float)(clock() - startTime) / 1000;
        elapsedTime = std::round(elapsedTime * 10) / 10; //Round to one decimal digit
        LOG(INFO) << "Filtering complete, took: " << elapsedTime << " seconds";
    
        return filteredColors;
    }

    NLMeansFilter::Buffer NLMeansFilter::EstimateVariance(Film * film, int weightSourceBuffer, int filterBuffer)
    {
        std::vector<std::vector<std::vector<Float>>> bufferMean0 = film->BufferMean(weightSourceBuffer);
        std::vector<std::vector<std::vector<Float>>> bufferMean1 = film->BufferMean(filterBuffer);

        std::vector<std::vector<std::vector<Float>>> bufferVariance0 = film->BufferVariance(weightSourceBuffer);
        std::vector<std::vector<std::vector<Float>>> bufferVariance1 = film->BufferVariance(filterBuffer);

        int sizeX = bufferMean0.size();
        int sizeY = bufferMean0[0].size();
        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                for (int i = 0; i < 3; i++)
                {
                    //Calculate the buffer differences
                    Float meanDifference = std::pow((bufferMean0[x][y][i] - bufferMean1[x][y][i]), 2);
                    Float varianceDifference = bufferVariance0[x][y][i] - bufferVariance1[x][y][i];

                    //Store differences in the buffers (former values will not be used anymore and can safely be overwritten)
                    bufferMean0[x][y][i] = meanDifference;
                    bufferVariance0[x][y][i] = varianceDifference;
                }
            }
        }

        Buffer filteredBufferDifference(bufferMean0.size(),                                    //size in 1st dimension
                                        std::vector<std::vector<Float>>(bufferMean0[0].size(), //size in 2nd dimension
                                        std::vector<Float>(3)));                               //3 color values for each pixel

        //bufferMean0 contains now the squared differences of color between buffers
        //bufferVariance0 contains now the differences of variance between buffers

        //The buffer variance differences are now used as the weight source and variance source to filter the squared buffer color differences 
        int previousFilterRadius = filterRadius;
        int previousRatchRadius = patchRadius;
        Float previousCancellationFactor = cancellationFactor;
        Float previousDampingFactor = dampingFactor;
        filterRadius = 1;
        patchRadius = 3;
        cancellationFactor = 4;
        dampingFactor = 0.45;
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredBufferDifference[x][y] = FilterPixel(Point2i(x, y), bufferVariance0, bufferVariance0, bufferMean0);
        filterRadius = previousFilterRadius;
        patchRadius = previousRatchRadius;
        cancellationFactor = previousCancellationFactor;
        dampingFactor = previousDampingFactor;

        //Clamp the filtered values to be no higher than the bufferVariance differences
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                    filteredBufferDifference[x][y][i] = std::min(filteredBufferDifference[x][y][i], bufferVariance0[x][y][i]);

        return filteredBufferDifference;
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

    NLMeansFilter::Buffer NLMeansFilter::DirectFilter (const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer)
    {
        Buffer filteredColors(filterBuffer.size(),                                    //size in 1st dimension
                              std::vector<std::vector<Float>>(filterBuffer[0].size(), //size in 2nd dimension
                              std::vector<Float>(3)));                                //3 color values for each pixel

        int sizeX = filterBuffer.size();
        int sizeY = filterBuffer[0].size();

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredColors[x][y] = FilterPixel(Point2i(x, y), weightSourceBuffer, weightSourceVarianceBuffer, filterBuffer);

        return filteredColors;
    }

    std::vector<Float> NLMeansFilter::FilterPixel (const Point2i &pixel, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Buffer &filterBuffer)
    {
        std::vector<Float> filteredColor(3);
        Bounds2i bounds = SharedBounds(weightSourceBuffer, std::vector<Point2i>{pixel}, filterRadius);

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offsetPixel = pixel + Point2i(x, y);
                Float weight = PatchWeight(weightSourceBuffer, pixel, offsetPixel, weightSourceVarianceBuffer);

                std::vector<Float> offsetColor = filterBuffer[offsetPixel.x][offsetPixel.y];
                for (int i = 0; i < 3; i++) filteredColor[i] += weight * offsetColor[i];

                sumOfWeights += weight;
            }
        }

        for (int i = 0; i < 3; i++) filteredColor[i] /= sumOfWeights;
        return filteredColor;
    }

    Float NLMeansFilter::PatchWeight (const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer)
    {
        Float patchDistance = PatchDistance(buffer, pixel1, pixel2, varianceBuffer);
        Float weight = std::exp(-std::max((Float)0, patchDistance));
        if (weight < 0.05) weight = 0; //Helps to preserve small details in noisy areas

        return weight;
    }

    Float NLMeansFilter::PatchDistance (const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer)
    {
        Bounds2i bounds = SharedBounds(buffer, std::vector<Point2i>{pixel1, pixel2}, patchRadius);

        Float sumOfDifferences;
        int pixelCounter = 0;

        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);

                std::vector<Float> pixelDistance = PixelDistance(buffer, pixel1 + offset, pixel2 + offset, varianceBuffer);
                sumOfDifferences += (pixelDistance[0] + pixelDistance[1] + pixelDistance[2]) / 3;

                pixelCounter += 1;
            }
        }

        Float average = sumOfDifferences / pixelCounter;
        return average;
    }

    std::vector<Float> NLMeansFilter::PixelDistance(const Buffer &buffer, const Point2i &pixel1, const Point2i &pixel2, const Buffer &varianceBuffer)
    {
        Float sqrDampingFactor = dampingFactor * dampingFactor;

        std::vector<Float> sqrDistance(3);
        std::vector<Float> cancellation(3);
        std::vector<Float> varianceSum(3);
        std::vector<Float> pixelDistance(3);
        for (int i = 0; i < 3; i++)
        {
            sqrDistance[i] = std::pow((buffer[pixel1.x][pixel1.y][i] - buffer[pixel2.x][pixel2.y][i]), 2);
            cancellation[i] = cancellationFactor * (varianceBuffer[pixel1.x][pixel1.y][i] + std::min(varianceBuffer[pixel1.x][pixel1.y][i], varianceBuffer[pixel2.x][pixel2.y][i]));
            varianceSum[i] = varianceBuffer[pixel1.x][pixel1.y][i] + varianceBuffer[pixel2.x][pixel2.y][i];

            pixelDistance[i] = (sqrDistance[i] - cancellation[i]) / (10e-10 + sqrDampingFactor * varianceSum[i]);
        }

        return pixelDistance;
    }

}
