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

        FillMapUniformly(sampleMap, iterationBudgets[currentIteration - 1]);

        //Before the last iteration all pixel values in the first buffer are set to 0, while the second buffer remains unchanged
        //Just a proof of concept for the multi buffer working correctly 
        //-> Result is an image of approximately half the brightness and effective samples than unaltered.
        if (currentIteration == plannedIterations)
        {
            for (Point2i position : film->croppedPixelBounds)
            {
                Pixel &pixel = film->GetPixel(0, position);
                pixel.xyz[0] = 0;
                pixel.xyz[1] = 0;
                pixel.xyz[2] = 0;
            }
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

}
