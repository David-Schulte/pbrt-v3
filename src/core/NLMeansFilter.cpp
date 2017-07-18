#include "NLMeansFilter.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{
    NLMeansFilter::NLMeansFilter() {}
    NLMeansFilter::~NLMeansFilter() {}

    void NLMeansFilter::SetParameters(int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor)
    {
        NLMeansFilter::filterRadius = filterRadius;
        NLMeansFilter::patchRadius = patchRadius;
        NLMeansFilter::cancellationFactor = cancellationFactor;
        NLMeansFilter::dampingFactor = dampingFactor;
        NLMeansFilter::sqrDampingFactor = std::pow(dampingFactor, 2);
    }

    void NLMeansFilter::ReserveParameters()
    {
        reservedFilterRadius = filterRadius;
        reservedPatchRadius = patchRadius;
        reservedCancellationFactor = cancellationFactor;
        reservedDampingFactor = dampingFactor;
    }

    void NLMeansFilter::RestoreParameters()
    {
        SetParameters(reservedFilterRadius, reservedPatchRadius, reservedCancellationFactor, reservedDampingFactor);
    }

    Float NLMeansFilter::FilterWeightSum(const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Point2i &pixel)
    {
        Bounds2i bounds = SharedBounds(weightSourceBuffer, pixel, filterRadius);

        std::vector<Float> tempStorage(3);
        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                Point2i contributingPixel = pixel + offset;
                Float weight = PatchWeight(weightSourceBuffer, weightSourceVarianceBuffer, pixel, contributingPixel, tempStorage);
                sumOfWeights += weight;
            }
        }

        return sumOfWeights;
    }

    NLMeansFilter::Buffer NLMeansFilter::Filter(Film * film, int filterSourceBuffer, int weightSourceBuffer)
    {
        Buffer filterSource = film->BufferXYZ(filterSourceBuffer);
        Buffer weightSource = film->BufferXYZ(weightSourceBuffer);
        Buffer varianceEstimation = EstimateVariance(film, filterSourceBuffer, weightSourceBuffer);

        film->WriteBufferImage("nlmeans varianceEstimation.exr", varianceEstimation); //Write estimated variance to file

        int sizeX = filterSource.size();
        int sizeY = filterSource[0].size();

        Buffer filteredColors(sizeX,                                 //size in 1st dimension
                              std::vector<std::vector<Float>>(sizeY, //size in 2nd dimension
                              std::vector<Float>(3)));               //3 color values for each pixel

        std::vector<Float> tempStorage(3);
        std::vector<Float> filteredPixel(3);

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
            {
                FilterPixel(filterSource, weightSource, varianceEstimation, Point2i(x, y), filteredPixel, tempStorage);
                for (int i = 0; i < 3; i++) filteredColors[x][y][i] = filteredPixel[i];
            }

        return filteredColors;
    }

    NLMeansFilter::Buffer NLMeansFilter::EstimateVariance(Film * film, int filterSourceBuffer, int weightSourceBuffer)
    {
        Buffer buffer0XYZ = film->BufferXYZ(filterSourceBuffer);
        Buffer buffer1XYZ = film->BufferXYZ(weightSourceBuffer);
        
        Buffer buffer0Variance = film->BufferVariance(filterSourceBuffer);
        Buffer buffer1Variance = film->BufferVariance(weightSourceBuffer);

        int sizeX = buffer0XYZ.size();
        int sizeY = buffer0XYZ[0].size();

        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                for (int i = 0; i < 3; i++)
                {
                    //Calculate color difference
                    Float colorDifference = buffer0XYZ[x][y][i] - buffer1XYZ[x][y][i];
                    colorDifference = std::pow(colorDifference, 2);
                    colorDifference /= 2;

                    //Calculate variance estimate == variance difference
                    Float varianceDifference = buffer0Variance[x][y][i] - buffer1Variance[x][y][i];
                    varianceDifference = std::abs(varianceDifference); //No negative values!

                    //Store differences in the buffers (former values will not be used anymore and can safely be overwritten)
                    buffer0XYZ[x][y][i] = colorDifference;
                    buffer0Variance[x][y][i] = varianceDifference;
                }
            }
        }

        //buffer0XYZ contains now the mean squared differences of color between buffers
        //buffer0Variance contains now the differences of variance between buffers
        //buffer1XYZ will later get overwritten with the filteredBufferDifference
        //Just renaming them here to clarify intent!
        Buffer &colorDifferenceBuffer = buffer0XYZ;
        Buffer &varianceDifferenceBuffer = buffer0Variance;
        Buffer &filteredBufferDifference = buffer1XYZ;

        //The buffer variance differences are now used as the weight source and variance source to filter the squared buffer color differences 
        //For this filtering process fixed parameters are used. The original values are restored afterwards.
        ReserveParameters();
        SetParameters(1, 3, 4, 0.45);

        std::vector<Float> tempStorage(3);
        std::vector<Float> filteredPixel(3);
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
            {
                FilterPixel(colorDifferenceBuffer, varianceDifferenceBuffer, varianceDifferenceBuffer, Point2i(x, y), filteredPixel, tempStorage);
                for (int i = 0; i < 3; i++) filteredBufferDifference[x][y][i] = filteredPixel[i];
            }

        RestoreParameters();

        //Clamp the filtered values to be no higher than the bufferVariance differences
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                    filteredBufferDifference[x][y][i] = std::min(filteredBufferDifference[x][y][i], varianceDifferenceBuffer[x][y][i]);

        return filteredBufferDifference;
    }

    void NLMeansFilter::FilterPixel (const Buffer &filterSourceBuffer, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Point2i &pixel, std::vector<Float> &resultStorage, std::vector<Float> &tempStorage)
    {
        Bounds2i bounds = SharedBounds(filterSourceBuffer, pixel, filterRadius);
        
        resultStorage[0] = 0;
        resultStorage[1] = 0;
        resultStorage[2] = 0;

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                Point2i contributingPixel = pixel + offset;

                Float weight = PatchWeight(weightSourceBuffer, weightSourceVarianceBuffer, pixel, contributingPixel, tempStorage);
                sumOfWeights += weight;

                for (int i = 0; i < 3; i++)
                {
                    Float contributingColor = filterSourceBuffer[contributingPixel.x][contributingPixel.y][i];
                    resultStorage[i] += contributingColor * weight;
                }
            }
        }

        for (int i = 0; i < 3; i++) resultStorage[i] /= sumOfWeights;
    }

    Float NLMeansFilter::PatchWeight (const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2, std::vector<Float> &tempStorage)
    {
        Float patchDistance = PatchDistance(buffer, varianceBuffer, pixel1, pixel2, tempStorage);
        if (patchDistance < 0) patchDistance = 0;

        Float weight = std::exp(-patchDistance);
        if (weight < 0.05) weight = 0; //Helps to preserve small details in noisy areas where many small weights can drown out few important ones

        return weight;
    }

    Float NLMeansFilter::PatchDistance (const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2, std::vector<Float> &tempStorage)
    {
        Bounds2i bounds = SharedBounds(buffer, pixel1, pixel2, patchRadius);

        Float sumOfDifferences;
        int pixelCounter = 0;

        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);

                PixelDistance(buffer, varianceBuffer, pixel1 + offset, pixel2 + offset, tempStorage);
                sumOfDifferences += (tempStorage[0] + tempStorage[1] + tempStorage[2]) / 3;

                pixelCounter += 1;
            }
        }

        Float average = sumOfDifferences / pixelCounter;
        return average;
    }

    void NLMeansFilter::PixelDistance(const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2, std::vector<Float> &resultStorage)
    {
        for (int i = 0; i < 3; i++)
        {
            Float sqrDistance = buffer[pixel1.x][pixel1.y][i] - buffer[pixel2.x][pixel2.y][i];
            sqrDistance = std::pow(sqrDistance, 2);

            Float variance1 = varianceBuffer[pixel1.x][pixel1.y][i];
            Float variance2 = varianceBuffer[pixel2.x][pixel2.y][i];
            Float minVariance = std::min(variance1, variance2);
            Float cancellation = cancellationFactor * (variance1 + minVariance);

            resultStorage[i] = (sqrDistance - cancellation) / (10e-10 + sqrDampingFactor * (variance1 + variance2));
        }
    }

    Bounds2i NLMeansFilter::SharedBounds(const Buffer &bounds, const Point2i &pixel, int radius)
    {
        return SharedBounds(bounds, pixel, pixel, radius);
    }

    Bounds2i NLMeansFilter::SharedBounds(const Buffer &bounds, const Point2i &pixel1, const Point2i &pixel2, int radius)
    {
        //Find the extent of all given pixels combined
        Bounds2i pixelExtents(pixel1, pixel1);

        pixelExtents.pMin.x = std::min(pixel1.x, pixel2.x);
        pixelExtents.pMin.y = std::min(pixel1.y, pixel2.y);
        pixelExtents.pMax.x = std::max(pixel1.x, pixel2.x);
        pixelExtents.pMax.y = std::max(pixel1.y, pixel2.y);

        //Find the biggest rectangle area, which can be placed upon every given pixel without overstepping the buffer bounds or the radius limit
        //The dimensions for this are are given relative to each pixel as offsets! Negative values are valid.
        Bounds2i sharedBounds;
        sharedBounds.pMin.x = -std::min(pixelExtents.pMin.x, radius);
        sharedBounds.pMin.y = -std::min(pixelExtents.pMin.y, radius);
        sharedBounds.pMax.x = std::min((int)bounds.size() - 1 - pixelExtents.pMax.x, radius);
        sharedBounds.pMax.y = std::min((int)bounds[0].size() - 1 - pixelExtents.pMax.y, radius);

        return sharedBounds;
    }

}
