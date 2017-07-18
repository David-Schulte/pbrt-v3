#include "NLMeansSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NLMeansSamplingPlanner::NLMeansSamplingPlanner(int initialBudget, int iterationBudget, int filterRadius, int patchRadius, Float cancellationFactor, Float dampingFactor) 
    { 
        NLMeansSamplingPlanner::initialBudget = initialBudget;
        NLMeansSamplingPlanner::iterationBudget = iterationBudget;
        filter = std::make_shared<NLMeansFilter>(); 
        filter->SetParameters(filterRadius, patchRadius, cancellationFactor, dampingFactor);
        LOG(INFO) << "NLMeansFilter filterRadius: " << filterRadius << ", patchRadius: " << patchRadius << ", cancellationFactor: " << cancellationFactor << ", dampingFactor: " << dampingFactor;
    }

    NLMeansSamplingPlanner::~NLMeansSamplingPlanner() {}

    void NLMeansSamplingPlanner::UpdateSampleMap(Film * film)
    {
        if (currentIteration == 1) return FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        DualBufferFiltering(film);

        Buffer estimatedError = EstimateError(film);
        FillMapErrorProportional(estimatedError);
        //FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        if (currentIteration > plannedIterations) //Post processing...
        {
            film->WriteBufferImage("nlmeans buffer1_pixelVariance.exr", film->BufferVariance(0));
            film->WriteBufferImage("nlmeans buffer2_pixelVariance.exr", film->BufferVariance(1));
            film->WriteBufferDifferenceImage("nlmeans bufferDifference.exr", 0, 1);

            film->SetBuffers(film->amountOfBuffers + 1);
            film->WriteToBuffer(estimatedError, film->amountOfBuffers - 1, 1);
            film->WriteBufferImage("nlmeans errorEstimation.exr", film->amountOfBuffers - 1);
            film->SetBuffers(film->amountOfBuffers - 1);
        }
    }

    void NLMeansSamplingPlanner::CreateSamplingPlan(Film *film)
    {
        film->SetBuffers(2);
        PlanIterations();
        PlanMaximalSamplesPerPixel();
    }

    void NLMeansSamplingPlanner::PlanIterations()
    {
        int budgetLeft = sampleBudgetPerPixel;
        iterationBudgets.clear();
        plannedIterations = 1;

        iterationBudgets.push_back(std::min(budgetLeft, initialBudget)); //Initial sampling will be done with the target budget unless the user specified to use less samples than that
        budgetLeft -= initialBudget;
        if (budgetLeft < 0) return; //If the budget is already exhausted stop here

        while (budgetLeft > iterationBudget)
        {
            plannedIterations += 1;
            iterationBudgets.push_back(iterationBudget);
            budgetLeft -= iterationBudget;
        }

        plannedIterations += 1;
        iterationBudgets.push_back(budgetLeft);
        return;
    }

    void NLMeansSamplingPlanner::PlanMaximalSamplesPerPixel()
    {
        maxTotalSamplesPerPixel = 0;
        for (int i = 0; i < iterationBudgets.size(); i++)
        {
            maxSampleBudgets.push_back(iterationBudgets[i] * 20);
            maxTotalSamplesPerPixel += maxSampleBudgets[i];
        }
    }

    void NLMeansSamplingPlanner::DualBufferFiltering(Film * film)
    {
        unsigned int startTime = clock();

        Buffer filteredBuffer0 = filter->Filter(film, 0, 1);
        Buffer filteredBuffer1 = filter->Filter(film, 1, 0);
        film->WriteToBuffer(filteredBuffer0, 0);
        film->WriteToBuffer(filteredBuffer1, 1);

        Float elapsedTime = (Float)(clock() - startTime) / 1000;
        elapsedTime = std::round(elapsedTime * 10) / 10; //Round to one decimal digit
        LOG(INFO) << "Dual buffer filtering complete. Took: " << elapsedTime << " seconds";
    }

    NLMeansSamplingPlanner::Buffer NLMeansSamplingPlanner::EstimateError(Film * film)
    {
        unsigned int startTime = clock();

        Buffer buffer0XYZ = film->BufferXYZ(0);
        Buffer buffer1XYZ = film->BufferXYZ(1);
        Buffer buffer0Error = film->BufferEmpty();
        Buffer buffer1Error = film->BufferEmpty();

        int sizeX = buffer0XYZ.size();
        int sizeY = buffer0XYZ[0].size();

        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                for (int i = 0; i < 3; i++)
                {
                    //Calculate the buffer color differences
                    Float squaredColorDifference = buffer0XYZ[x][y][i] - buffer1XYZ[x][y][i];
                    squaredColorDifference = std::pow(squaredColorDifference, 2);

                    //Calculate the variance cancellation values
                    Float squaredColor0 = std::pow(buffer0XYZ[x][y][i], 2);
                    Float squaredColor1 = std::pow(buffer1XYZ[x][y][i], 2);

                    //Calculate the variance cancelled error values for the buffers
                    Float error0 = squaredColorDifference / (squaredColor0 + 10e-3);
                    Float error1 = squaredColorDifference / (squaredColor1 + 10e-3);

                    //Store the average error in buffer0XYZ, which can now be safely overwritten
                    buffer0Error[x][y][i] = error0;
                    buffer1Error[x][y][i] = error1;
                }
            }
        }

        //Weight all the estimated values by how much contribution they give to their surrounding pixels and store result in bufferMean0's first Float value
        Buffer buffer0Variance = film->BufferVariance(0);
        Buffer buffer1Variance = film->BufferVariance(1);
        Buffer buffer0Weights = film->BufferWeights(0);
        Buffer buffer1Weights = film->BufferWeights(1);
        Buffer weightStorage = film->BufferEmpty();

        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                Float weight0 = filter->FilterWeightSum(buffer0XYZ, buffer0Variance, Point2i(x, y)); //TODO:: Are these buffers the right ones to use?
                Float weight1 = filter->FilterWeightSum(buffer1XYZ, buffer1Variance, Point2i(x, y)); //TODO:: Are these buffers the right ones to use?
                weight0 /= (buffer0Weights[x][y][0] + 1);
                weight1 /= (buffer1Weights[x][y][0] + 1);

                //Apply the calculated weights to the buffer errors
                for (int i = 0; i < 3; i++)
                {
                    buffer0Error[x][y][i] *= weight0;
                    buffer1Error[x][y][i] *= weight1;
                }
            }
        }

        //Average the buffers error to obtain a single error value which can be used to drive the sampling process
        Buffer averageBufferError = film->BufferEmpty();
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                    averageBufferError[x][y][i] = (buffer0Error[x][y][i] + buffer1Error[x][y][i]) / 2;

        Float elapsedTime = (Float)(clock() - startTime) / 1000;
        elapsedTime = std::round(elapsedTime * 10) / 10; //Round to one decimal digit
        LOG(INFO) << "Error estimation complete, took: " << elapsedTime << " seconds";

        return averageBufferError;
    }

    void NLMeansSamplingPlanner::FillMapErrorProportional(const Buffer &error)
    {
        int sizeX = sampleMap.size();
        int sizeY = sampleMap[0].size();

        Float averageError = 0;
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                averageError += (error[x][y][0] + error[x][y][1] + error[x][y][2]) / 3;
        averageError = averageError / sizeX * sizeY;

        int averageFreePixelBudget = iterationBudgets[currentIteration - 1] - 1;
        Float overflow = 0;
        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                Float currentError = (error[x][y][0] + error[x][y][1] + error[x][y][2]) / 3;
                Float errorWeight = currentError / averageError;
                Float proportionalSamplingRate = errorWeight * averageFreePixelBudget;
                int flooredSamplingRate = std::floor(proportionalSamplingRate);

                overflow += proportionalSamplingRate - flooredSamplingRate;
                if (overflow >= 1) //Mathematically ensures that the whole sample budget in this iteration is used up exactly. In reality the last sample may be lost due to inaccurities adding up.
                {
                    flooredSamplingRate += 1;
                    overflow -= 1;
                }

                //TODO:: Clamping values which are too high is not getting compensated. If clamping occurs, the effective sampling budget will be lower than specified by the user
                int clampedSamplingRate = std::min(maxSampleBudgets[currentIteration - 1], flooredSamplingRate + 1);
                sampleMap[x][y] = clampedSamplingRate;
            }
        }
    }

}
