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
        Bounds2i bounds = SharedBounds(weightSourceBuffer, std::vector<Point2i>{pixel}, filterRadius);

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                Point2i contributingPixel = pixel + offset;
                Float weight = PatchWeight(weightSourceBuffer, weightSourceVarianceBuffer, pixel, contributingPixel);
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

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredColors[x][y] = FilterPixel(filterSource, weightSource, varianceEstimation, Point2i(x, y));

        return filteredColors;
    }

    NLMeansFilter::Buffer NLMeansFilter::EstimateVariance(Film * film, int filterSourceBuffer, int weightSourceBuffer)
    {
        std::vector<std::vector<std::vector<Float>>> buffer0XYZ = film->BufferXYZ(filterSourceBuffer);
        std::vector<std::vector<std::vector<Float>>> buffer1XYZ = film->BufferXYZ(weightSourceBuffer);
        
        std::vector<std::vector<std::vector<Float>>> buffer0Variance = film->BufferVariance(filterSourceBuffer);
        std::vector<std::vector<std::vector<Float>>> buffer1Variance = film->BufferVariance(weightSourceBuffer);

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

        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                filteredBufferDifference[x][y] = FilterPixel(colorDifferenceBuffer, varianceDifferenceBuffer, varianceDifferenceBuffer, Point2i(x, y));

        RestoreParameters();

        //Clamp the filtered values to be no higher than the bufferVariance differences
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                    filteredBufferDifference[x][y][i] = std::min(filteredBufferDifference[x][y][i], varianceDifferenceBuffer[x][y][i]);

        return filteredBufferDifference;
    }

    std::vector<Float> NLMeansFilter::FilterPixel (const Buffer &filterSourceBuffer, const Buffer &weightSourceBuffer, const Buffer &weightSourceVarianceBuffer, const Point2i &pixel)
    {
        std::vector<Float> filteredColor(3);
        Bounds2i bounds = SharedBounds(filterSourceBuffer, std::vector<Point2i>{pixel}, filterRadius);

        Float sumOfWeights = 0;
        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);
                Point2i contributingPixel = pixel + offset;
                std::vector<Float> contributingColor = filterSourceBuffer[contributingPixel.x][contributingPixel.y];

                Float weight = PatchWeight(weightSourceBuffer, weightSourceVarianceBuffer, pixel, contributingPixel);
                sumOfWeights += weight;

                for (int i = 0; i < 3; i++) filteredColor[i] += contributingColor[i] * weight;
            }
        }

        for (int i = 0; i < 3; i++) filteredColor[i] /= sumOfWeights;
        return filteredColor;
    }

    Float NLMeansFilter::PatchWeight (const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2)
    {
        Float patchDistance = PatchDistance(buffer, varianceBuffer, pixel1, pixel2);
        if (patchDistance < 0) patchDistance = 0;

        Float weight = std::exp(-patchDistance);
        if (weight < 0.05) weight = 0; //Helps to preserve small details in noisy areas where many small weights can drown out few important ones

        return weight;
    }

    Float NLMeansFilter::PatchDistance (const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2)
    {
        Bounds2i bounds = SharedBounds(buffer, std::vector<Point2i>{pixel1, pixel2}, patchRadius);

        Float sumOfDifferences;
        int pixelCounter = 0;

        for (int y = bounds.pMin.y; y <= bounds.pMax.y; y++)
        {
            for (int x = bounds.pMin.x; x <= bounds.pMax.x; x++)
            {
                Point2i offset(x, y);

                std::vector<Float> pixelDistance = PixelDistance(buffer, varianceBuffer, pixel1 + offset, pixel2 + offset);
                sumOfDifferences += (pixelDistance[0] + pixelDistance[1] + pixelDistance[2]) / 3;

                pixelCounter += 1;
            }
        }

        Float average = sumOfDifferences / pixelCounter;
        return average;
    }

    std::vector<Float> NLMeansFilter::PixelDistance(const Buffer &buffer, const Buffer &varianceBuffer, const Point2i &pixel1, const Point2i &pixel2)
    {
        Float sqrDampingFactor = dampingFactor * dampingFactor;

        std::vector<Float> sqrDistance(3);
        std::vector<Float> cancellation(3);
        std::vector<Float> pixelDistance(3);
        for (int i = 0; i < 3; i++)
        {
            sqrDistance[i] = buffer[pixel1.x][pixel1.y][i] - buffer[pixel2.x][pixel2.y][i];
            sqrDistance[i] = sqrDistance[i] * sqrDistance[i];

            Float variance1 = varianceBuffer[pixel1.x][pixel1.y][i];
            Float variance2 = varianceBuffer[pixel2.x][pixel2.y][i];
            Float minVariance = std::min(variance1, variance2);
            cancellation[i] = cancellationFactor * (variance1 + minVariance);

            pixelDistance[i] = (sqrDistance[i] - cancellation[i]) / (10e-10 + sqrDampingFactor * (variance1 + variance2));
        }

        return pixelDistance;
    }

    Bounds2i NLMeansFilter::SharedBounds(const Buffer &bounds, const std::vector<Point2i> &pixels, int radius)
    {
        //Find the extent of all given pixels combined
        Bounds2i pixelExtents(pixels[0], pixels[0]);
        for (int i = 1; i < pixels.size(); i++) //First pixel can be excluded since the extent is initialized with its values
        {
            pixelExtents.pMin.x = std::min(pixels[i].x, pixelExtents.pMin.x);
            pixelExtents.pMin.y = std::min(pixels[i].y, pixelExtents.pMin.y);
            pixelExtents.pMax.x = std::max(pixels[i].x, pixelExtents.pMax.x);
            pixelExtents.pMax.y = std::max(pixels[i].y, pixelExtents.pMax.y);
        }

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
