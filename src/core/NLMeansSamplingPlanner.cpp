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

        FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);
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
        float filterRadius = std::ceil(std::sqrt(minDimension) / 5);
        float patchRadius = std::ceil(filterRadius / 2);

        std::vector<std::vector<std::vector<float>>> filteredBuffer0 = filter->Filter(film, 1, 0, filterRadius, patchRadius);
        std::vector<std::vector<std::vector<float>>> filteredBuffer1 = filter->Filter(film, 0, 1, filterRadius, patchRadius);
        
        for (Point2i pixel : film->croppedPixelBounds)
        {
            Pixel& buffer0Current = film->GetPixel(0, pixel);
            for (int i = 0; i < 3; i++) buffer0Current.xyz[i] = filteredBuffer0[pixel.x][pixel.y][i] * buffer0Current.filterWeightSum;
        
            Pixel& buffer1Current = film->GetPixel(1, pixel);
            for (int i = 0; i < 3; i++) buffer1Current.xyz[i] = filteredBuffer1[pixel.x][pixel.y][i] * buffer1Current.filterWeightSum;
        }
    }

}
