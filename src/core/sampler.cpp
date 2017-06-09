
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// core/sampler.cpp*
#include "sampler.h"
#include "sampling.h"
#include "camera.h"
#include "stats.h"

namespace pbrt {

// Sampler Method Definitions
Sampler::~Sampler() {}

Sampler::Sampler(int64_t samplesPerPixel) : averagePerPixelSampleBudget(samplesPerPixel), maxSamplesPerPixel(16){ printf("AveragePerPixelSampleBudget: %i\n", averagePerPixelSampleBudget); } // Debug!

CameraSample Sampler::GetCameraSample(const Point2i &pRaster) 
{
    CameraSample cs;
    cs.pFilm = (Point2f)pRaster + Get2D();
    cs.time = Get1D();
    cs.pLens = Get2D();

	if (currentPixel.x == 750 && currentPixel.y == 750)
	{
		printf("\n Camera sample values:\n");
		printf("\t cs.pFilm: [%f , %f ]\n", cs.pFilm.x, cs.pFilm.y);
		printf("\t cs.time: %i\n", cs.time);
		printf("\t cs.pLens: [ %f , %f ]\n\n", cs.pLens.x, cs.pLens.y);
	}

    return cs;
}

void Sampler::StartPixel(const Point2i &p) 
{
    currentPixel = p;
    //currentPixelSampleIndex = 0;
	currentPixelSampleIndex = samplingPlanner->CurrentSampleNumber(p);
	//if (currentPixelSampleIndex>8)
	//{
	//	printf("CurrentPixelSampleIndex: %i\n", currentPixelSampleIndex);
	//}
	
    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
}

bool Sampler::StartNextSample() 
{
    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
	++currentPixelSampleIndex;
	//if(currentPixelSampleIndex>8)
		//printf("CurrentPixelSampleIndex: %i\n", currentPixelSampleIndex);
	return (currentPixelSampleIndex < samplingPlanner->PlannedSamples(currentPixel) + samplingPlanner->CurrentSampleNumber(currentPixel)) && currentPixelSampleIndex < averagePerPixelSampleBudget;
}

void Sampler::InitializeSamplingPlan(Film *film)
{
    if (samplingPlanner == nullptr) LOG(FATAL) << "Sampler does not have a samplingPlanner";

    samplingPlanner->InitializeSamplingPlan(averagePerPixelSampleBudget, film);
    //maxSamplesPerPixel = samplingPlanner->maxPixelSamplesPerIteration;
    AdaptToSamplingPlan();
}

bool Sampler::StartNextIteration()
{
    return samplingPlanner->StartNextIteration();
}

void Sampler::UpdateSamplingPlan(Film * film, const int64_t adaptiveSamplesCount)
{
    samplingPlanner->UpdateSamplingPlan(film, adaptiveSamplesCount);
}

void Sampler::UpdateCurrentSampleNumberMap()
{
	samplingPlanner->UpdateCurrentSampleNumberMap();
}

bool Sampler::SetSampleNumber(int64_t sampleNum) {
    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
    currentPixelSampleIndex = sampleNum;
    return currentPixelSampleIndex < averagePerPixelSampleBudget;
}

void Sampler::Request1DArray(int n) {
    CHECK_EQ(RoundCount(n), n);
    samples1DArraySizes.push_back(n);
    sampleArray1D.push_back(std::vector<Float>(n * averagePerPixelSampleBudget));
}

void Sampler::Request2DArray(int n) {
    CHECK_EQ(RoundCount(n), n);
    samples2DArraySizes.push_back(n);
    sampleArray2D.push_back(std::vector<Point2f>(n * averagePerPixelSampleBudget));
}

const Float *Sampler::Get1DArray(int n) {
    if (array1DOffset == sampleArray1D.size()) return nullptr;
    CHECK_EQ(samples1DArraySizes[array1DOffset], n);
    CHECK_LT(currentPixelSampleIndex, averagePerPixelSampleBudget);
    return &sampleArray1D[array1DOffset++][currentPixelSampleIndex * n];
}

const Point2f *Sampler::Get2DArray(int n) {
    if (array2DOffset == sampleArray2D.size()) return nullptr;
    CHECK_EQ(samples2DArraySizes[array2DOffset], n);
    CHECK_LT(currentPixelSampleIndex, averagePerPixelSampleBudget);
    return &sampleArray2D[array2DOffset++][currentPixelSampleIndex * n];
}

PixelSampler::PixelSampler(int64_t samplesPerPixel, int nSampledDimensions)
    : Sampler(samplesPerPixel) 
{
    samplingDimensions = nSampledDimensions;

    for (int i = 0; i < samplingDimensions; ++i)
    {
        samples1D.push_back(std::vector<Float>(samplesPerPixel));
        samples2D.push_back(std::vector<Point2f>(samplesPerPixel));
    }
}

bool PixelSampler::StartNextSample() {
    current1DDimension = current2DDimension = 0;
    return Sampler::StartNextSample();
}

bool PixelSampler::SetSampleNumber(int64_t sampleNum) {
    current1DDimension = current2DDimension = 0;
    return Sampler::SetSampleNumber(sampleNum);
}

Float PixelSampler::Get1D() {
    ProfilePhase _(Prof::GetSample);
    CHECK_LT(currentPixelSampleIndex, averagePerPixelSampleBudget);
    if (current1DDimension < samples1D.size())
        return samples1D[current1DDimension++][currentPixelSampleIndex];
    else
        return rng.UniformFloat();
}

Point2f PixelSampler::Get2D() {
    ProfilePhase _(Prof::GetSample);
    CHECK_LT(currentPixelSampleIndex, averagePerPixelSampleBudget);
    if (current2DDimension < samples2D.size())
        return samples2D[current2DDimension++][currentPixelSampleIndex];
    else
        return Point2f(rng.UniformFloat(), rng.UniformFloat());
}

void PixelSampler::AdaptToSamplingPlan()
{
    samples1D.clear();
    samples2D.clear();

    for (int i = 0; i < samplingDimensions; ++i) 
    {
        samples1D.push_back(std::vector<Float>(samplingPlanner->maxPixelSamplesPerIteration));
        samples2D.push_back(std::vector<Point2f>(samplingPlanner->maxPixelSamplesPerIteration));
    }
}

void GlobalSampler::StartPixel(const Point2i &p) {
    ProfilePhase _(Prof::StartPixel);
    Sampler::StartPixel(p);
    dimension = 0;
    //intervalSampleIndex = GetIndexForSample(0);
	intervalSampleIndex = GetIndexForSample(samplingPlanner->CurrentSampleNumber(p));
    // Compute _arrayEndDim_ for dimensions used for array samples
    arrayEndDim =
        arrayStartDim + sampleArray1D.size() + 2 * sampleArray2D.size();

    // Compute 1D array samples for _GlobalSampler_
    for (size_t i = 0; i < samples1DArraySizes.size(); ++i) {
        int nSamples = samples1DArraySizes[i] * averagePerPixelSampleBudget;
        for (int j = 0; j < nSamples; ++j) {
            int64_t index = GetIndexForSample(j);
            sampleArray1D[i][j] = SampleDimension(index, arrayStartDim + i);
        }
    }

    // Compute 2D array samples for _GlobalSampler_
    int dim = arrayStartDim + samples1DArraySizes.size();
    for (size_t i = 0; i < samples2DArraySizes.size(); ++i) {
        int nSamples = samples2DArraySizes[i] * averagePerPixelSampleBudget;
        for (int j = 0; j < nSamples; ++j) {
            int64_t idx = GetIndexForSample(j);
            sampleArray2D[i][j].x = SampleDimension(idx, dim);
            sampleArray2D[i][j].y = SampleDimension(idx, dim + 1);
        }
        dim += 2;
    }
    CHECK_EQ(arrayEndDim, dim);
}

bool GlobalSampler::StartNextSample() {
    dimension = 0;
    intervalSampleIndex = GetIndexForSample(currentPixelSampleIndex + 1);

	//int dimension;
	//int64_t intervalSampleIndex;
	//static const int arrayStartDim = 5;
	//int arrayEndDim;

	if (currentPixel.x == 750 && currentPixel.y == 750)
	{
		printf("\n Global sampler member variables:\n");
		printf("\t dimension: %i\n", dimension);
		printf("\t intervalSampleIndex: %i\n", intervalSampleIndex);
		printf("\t arrayStartDim: %i\n", arrayStartDim);
		printf("\t arrayEndDim: %i\n\n", arrayEndDim);
	}

    return Sampler::StartNextSample();
}

bool GlobalSampler::SetSampleNumber(int64_t sampleNum) {
    dimension = 0;
    intervalSampleIndex = GetIndexForSample(sampleNum);
    return Sampler::SetSampleNumber(sampleNum);
}

Float GlobalSampler::Get1D() {
    ProfilePhase _(Prof::GetSample);
    if (dimension >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
	dimension++;
	//printf("Dimension in 1D: %i\n", dimension);
    return SampleDimension(intervalSampleIndex, dimension);
}

Point2f GlobalSampler::Get2D() {
    ProfilePhase _(Prof::GetSample);
    if (dimension + 1 >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    Point2f p(SampleDimension(intervalSampleIndex, dimension),
              SampleDimension(intervalSampleIndex, dimension + 1));
    dimension += 2;
	//printf("Dimension in 2D: %i\n", dimension);
    return p;
}

}  // namespace pbrt
