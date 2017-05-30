#include "SamplingPlanner.h"
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt 
{

    SamplingPlanner::SamplingPlanner() {}
    SamplingPlanner::~SamplingPlanner() {}

    void SamplingPlanner::Initialize(int samplesPerPixel, Film * film)
    {
        currentIteration = 1;
        sampleBudgetPerPixel = samplesPerPixel;

        Bounds2i filmBounds = film->croppedPixelBounds;
        filmWidth = filmBounds.pMax.x - filmBounds.pMin.x;
        filmHeight = filmBounds.pMax.y - filmBounds.pMin.y;
        totalSampleBudget = sampleBudgetPerPixel * filmWidth * filmHeight;

        CreateSampleMap(film);
        CreateSamplingPlan(film);
    }

    void SamplingPlanner::CreateSampleMap(Film * film)
    {
        Bounds2i sampleBounds = film->GetSampleBounds();
        Vector2i sampleExtent = sampleBounds.Diagonal();
        sampleMap = std::vector<std::vector<int>>(sampleExtent.x, std::vector<int>(sampleExtent.y));
    }

    void SamplingPlanner::FillMapUniformly(int samplesPerPixel)
    {
        for (int column = 0; column < sampleMap.size(); column++)
            for (int row = 0; row < sampleMap[0].size(); row++)
                sampleMap[column][row] = samplesPerPixel;
    }

    bool SamplingPlanner::StartNextIteration()
    {
        currentIteration++;

        if (currentIteration > plannedIterations) return false;
        else                                      return true;
    }

}