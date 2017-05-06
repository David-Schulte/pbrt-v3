// integrators/AdaptiveNLMeansIntegrator.h*
#include "integrators/AdaptiveNLMeansIntegrator.h"

/*
Input: InitialSamples           //Per pixel
Input: AdaptiveSampleBudget     //Per pixel

Method: Convert AdaptiveSampleBudget to total samples   //Individual pixels

Method: InitializeBasis
    Method: UniformSampling     //Only sample pixels uniformly
    Method: Filter
    Method: EstimateError       //Save error as grayscale image -> stores adaptive samples needed

while (AdaptiveSampleBudget > 0):
    Method: AdaptiveImprove
        Method: AdaptiveSampling    //Sample pixels corresponding to the error values until AdaptiveSampleBudget is exhausted
        Method: Filter
        Method: EstimateError
*/

std::vector<double> AdaptiveNLMeansIntegrator::CreateSampleMap() const
{
    return std::vector<double>();
}
