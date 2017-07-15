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

        EstimateError(film);
        //FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        if (currentIteration > plannedIterations) //Post processing...
        {
            film->WriteBufferImage("buffer1.exr", 0);
            film->WriteBufferImage("buffer2.exr", 1);
            film->WriteVarianceImage("pixelVariance.exr", 0);
            film->WriteBufferDifferenceImage("bufferDifference.exr", 0, 1);
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
        maxSamplesPerPixel = plannedIterations * iterationBudgetTarget * 20;
    }

    void NLMeansSamplingPlanner::DualBufferFiltering(Film * film)
    {
        int minDimension = std::min(film->croppedPixelBounds.pMax.x - film->croppedPixelBounds.pMin.x, 
                                    film->croppedPixelBounds.pMax.y - film->croppedPixelBounds.pMin.y);
        float filterRadius = std::ceil(std::sqrt(minDimension) / 10);
        float patchRadius = std::ceil(filterRadius / 2);

        LOG(INFO) << "filterRadius: " << filterRadius << ", patchRadius: " << patchRadius;

        std::vector<std::vector<std::vector<float>>> filteredBuffer0 = filter->Filter(film, 1, 0, filterRadius, patchRadius);
        std::vector<std::vector<std::vector<float>>> filteredBuffer1 = filter->Filter(film, 0, 1, filterRadius, patchRadius);
        
        for (Point2i position : film->croppedPixelBounds)
        {
            Pixel& pixel1 = film->GetPixel(0, position);
            for (int i = 0; i < 3; i++) pixel1.xyz[i] = filteredBuffer0[position.x][position.y][i] * pixel1.filterWeightSum;
        
            Pixel& pixel2 = film->GetPixel(1, position);
            for (int i = 0; i < 3; i++) pixel2.xyz[i] = filteredBuffer1[position.x][position.y][i] * pixel2.filterWeightSum;
        }
    }

    void NLMeansSamplingPlanner::EstimateError(Film * film)
    {
        std::vector<std::vector<float>> bufferDifference(film->croppedPixelBounds.pMax.x - film->croppedPixelBounds.pMin.x,
                                                         std::vector<float>(film->croppedPixelBounds.pMax.y - film->croppedPixelBounds.pMin.y));

        int pixelCounter = 0;
        float averageDifference = 0;
        for (Point2i position : film->croppedPixelBounds)
        {
            Pixel& pixel1 = film->GetPixel(0, position);
            Pixel& pixel2 = film->GetPixel(1, position);

            for (int i = 0; i < 3; i++)
            {
                float channelDifference = (pixel1.xyz[i] / pixel1.filterWeightSum) - (pixel2.xyz[i] / pixel2.filterWeightSum);
                channelDifference = std::abs(channelDifference);
                bufferDifference[position.x][position.y] += channelDifference;
            }
            bufferDifference[position.x][position.y] /= 3;

            averageDifference += bufferDifference[position.x][position.y];
            pixelCounter += 1;
        }
        averageDifference /= pixelCounter;

        LOG(INFO) << "Average color difference of buffers: " << averageDifference;

        int averageFreePixelBudget = iterationBudgets[currentIteration - 1] - 1;
        float overflow = 0;
        for (Point2i position : film->croppedPixelBounds)
        {
            float errorWeight = bufferDifference[position.x][position.y] / averageDifference;
            float proportionalSamplingRate = errorWeight * averageFreePixelBudget;
            int flooredSamplingRate = std::floor(proportionalSamplingRate);

            overflow += proportionalSamplingRate - flooredSamplingRate;
            if (overflow >= 1) //Mathematically ensures that the whole sample budget in this iteration is used up exactly. In reality the last sample may be lost due to inaccurities adding up.
            {
                flooredSamplingRate += 1;
                overflow -= 1;
            }

            //TODO:: A single pixel could potentially eat up all free samples of the whole iteration -> should be limited to a max value (see maxSamplesPerPixel)...
            sampleMap[position.x][position.y] = flooredSamplingRate + 1; //Use always at least one sample
        }
    }

}
