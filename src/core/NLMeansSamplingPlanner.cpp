#include "NLMeansSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NLMeansSamplingPlanner::NLMeansSamplingPlanner() {}
    NLMeansSamplingPlanner::~NLMeansSamplingPlanner() {}

    void NLMeansSamplingPlanner::UpdateSampleMap(Film * film)
    {
        if (currentIteration == 1) return FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        DualBufferFiltering(film);

        std::vector<std::vector<std::vector<Float>>> estimatedError = EstimateError(film);
        FillMapErrorProportional(estimatedError);
        //FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        if (currentIteration > plannedIterations) //Post processing...
        {
            film->WriteBufferImage("buffer1.exr", 0);
            film->WriteBufferImage("buffer2.exr", 1);
            film->WriteVarianceImage("buffer1_pixelVariance.exr", 0);
            film->WriteVarianceImage("buffer2_pixelVariance.exr", 1);
            film->WriteBufferDifferenceImage("bufferDifference.exr", 0, 1);

            film->SetBuffers(film->amountOfBuffers + 1);
            film->WriteToBuffer(EstimateError(film), film->amountOfBuffers - 1, 1);
            film->WriteBufferImage("errorEstimation.exr", film->amountOfBuffers - 1);
            film->SetBuffers(film->amountOfBuffers - 1);
        }
    }

    void NLMeansSamplingPlanner::CreateSamplingPlan(Film *film)
    {
        filter = std::make_shared<NLMeansFilter>();
        film->SetBuffers(2);
        PlanIterations();
        PlanMaximalSamplesPerPixel();
    }

    void NLMeansSamplingPlanner::PlanIterations()
    {
        int budgetLeft = sampleBudgetPerPixel;
        iterationBudgets.clear();
        plannedIterations = 1;

        iterationBudgets.push_back(std::min(budgetLeft, initialBudgetTarget)); //Initial sampling will be done with the target budget unless the user specified to use less samples than that
        budgetLeft -= initialBudgetTarget;
        if (budgetLeft < 0) return; //If the budget is already exhausted stop here

        while (budgetLeft > iterationBudgetTarget)
        {
            plannedIterations += 1;
            iterationBudgets.push_back(iterationBudgetTarget);
            budgetLeft -= iterationBudgetTarget;
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
        int minDimension = std::min(film->croppedPixelBounds.pMax.x - film->croppedPixelBounds.pMin.x, 
                                    film->croppedPixelBounds.pMax.y - film->croppedPixelBounds.pMin.y);
        float filterRadius = std::ceil(std::sqrt(minDimension) / 10);
        float patchRadius = std::ceil(filterRadius / 2);

        LOG(INFO) << "filterRadius: " << filterRadius << ", patchRadius: " << patchRadius;

        std::vector<std::vector<std::vector<Float>>> filteredBuffer0 = filter->Filter(film, 1, 0, filterRadius, patchRadius, 1, 0.4);
        std::vector<std::vector<std::vector<Float>>> filteredBuffer1 = filter->Filter(film, 0, 1, filterRadius, patchRadius, 1, 0.4);
        film->WriteToBuffer(filteredBuffer0, 0);
        film->WriteToBuffer(filteredBuffer1, 1);
    }

    std::vector<std::vector<std::vector<Float>>> NLMeansSamplingPlanner::EstimateError(Film * film)
    {
        unsigned int startTime = clock();

        std::vector<std::vector<std::vector<Float>>> bufferMean0 = film->BufferMean(0);
        std::vector<std::vector<std::vector<Float>>> bufferMean1 = film->BufferMean(1);
        std::vector<std::vector<std::vector<Float>>> bufferMeanDifference = film->BufferMean(0);

        int sizeX = bufferMeanDifference.size();
        int sizeY = bufferMeanDifference[0].size();
        int pixelCounter = 0;
        Float averageMeanDifference = 0;

        for (int y = 0; y < sizeY; y++)
        {
            for (int x = 0; x < sizeX; x++)
            {
                for (int i = 0; i < 3; i++)
                {
                    //Calculate the buffer differences
                    Float meanDifference = std::pow((bufferMean0[x][y][i] - bufferMean1[x][y][i]), 2);

                    //Store differences in the buffers (former values will not be used anymore and can safely be overwritten)
                    bufferMeanDifference[x][y][i] = meanDifference;

                    //Sum values up to get an average later
                    averageMeanDifference += meanDifference;
                }
                pixelCounter += 1;
            }
        }
        LOG(INFO) << "Average squared color difference of buffers: " << averageMeanDifference / pixelCounter;

        //Insert variance canceled sqr differences into the fromer bufferMean vectors, replacing their content
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                {
                    bufferMean0[x][y][i] = bufferMeanDifference[x][y][i] / (0.0001 + std::pow(bufferMean0[x][y][i], 2));
                    bufferMean1[x][y][i] = bufferMeanDifference[x][y][i] / (0.0001 + std::pow(bufferMean1[x][y][i], 2));
                }

        //Insert the average buffer error (now stored in the bufferMean's) into bufferMeanDifference 
        for (int y = 0; y < sizeY; y++)
            for (int x = 0; x < sizeX; x++)
                for (int i = 0; i < 3; i++)
                    bufferMeanDifference[x][y][i] = (bufferMean0[x][y][i] + bufferMean1[x][y][i]) / 2;


        Float elapsedTime = (Float)(clock() - startTime) / 1000;
        elapsedTime = std::round(elapsedTime * 10) / 10; //Round to one decimal digit
        LOG(INFO) << "Error estimation complete, took: " << elapsedTime << " seconds";

        return bufferMeanDifference;
    }

    void NLMeansSamplingPlanner::FillMapErrorProportional(const std::vector<std::vector<std::vector<Float>>> &error)
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
