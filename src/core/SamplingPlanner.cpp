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

    void SamplingPlanner::UpdateSamplingPlan(Film * film)
    {
        if (currentIteration > 1) AddSampleMapToDistribution();

        UpdateSampleMap(film);
    }

    void SamplingPlanner::CreateSampleMap(Film * film)
    {
        Bounds2i sampleBounds = film->GetSampleBounds();
        Vector2i sampleExtent = sampleBounds.Diagonal();
        sampleMap = std::vector<std::vector<int>>(sampleExtent.x, std::vector<int>(sampleExtent.y));
        totalDistribution = std::vector<std::vector<int>>(sampleExtent.x, std::vector<int>(sampleExtent.y));
        FillMapUniformly(sampleMap, 0);
        FillMapUniformly(totalDistribution, 0);
    }

    void SamplingPlanner::FillMapUniformly(std::vector<std::vector<int>> & map,  int value)
    {
        for (int column = 0; column < map.size(); column++)
            for (int row = 0; row < map[0].size(); row++)
                map[column][row] = value;
    }

    bool SamplingPlanner::StartNextIteration()
    {
        currentIteration++;

        if (currentIteration > plannedIterations) return false;
        else                                      return true;
    }

    int SamplingPlanner::SamplesOfPreviousIterations(const Point2i & pixel)
    {
        return totalDistribution[pixel.x][pixel.y];
    }

    int SamplingPlanner::PlannedSamplesForThisIteration(const Point2i & pixel)
    {
        return sampleMap[pixel.x][pixel.y];
    }

    void SamplingPlanner::AddSampleMapToDistribution()
    {
        for (int column = 0; column < sampleMap.size(); column++)
            for (int row = 0; row < sampleMap[0].size(); row++)
                totalDistribution[column][row] += sampleMap[column][row];
    }

}