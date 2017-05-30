#include "NLMeansSamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt
{

    NLMeansSamplingPlanner::NLMeansSamplingPlanner() {}
    NLMeansSamplingPlanner::~NLMeansSamplingPlanner() {}

    void NLMeansSamplingPlanner::UpdateSamplingPlan(Film * film)
    {
        //if (currentIteration == 1) return FillMapUniformly(iterationBudgets[currentIteration - 1]);
        //
        //FillMapUniformly(iterationBudgets[currentIteration - 1]);

        for (int column = 0; column < sampleMap.size(); column++)
        {
            for (int row = 0; row < sampleMap[0].size(); row++)
            {
                if (row < 600) sampleMap[column][row] = 1;
                else           sampleMap[column][row] = 50;
            }
        }
    }

    void NLMeansSamplingPlanner::CreateSamplingPlan(Film *film)
    {
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
        maxPixelSamplesPerIteration = 51;
    }

}
