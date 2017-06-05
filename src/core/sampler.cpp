
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

Sampler::Sampler(int64_t samplesPerPixel) : averagePerPixelSampleBudget(samplesPerPixel) {}

CameraSample Sampler::GetCameraSample(const Point2i &pRaster) 
{
    CameraSample cs;
    cs.pFilm = (Point2f)pRaster + Get2D();
    cs.time = Get1D();
    cs.pLens = Get2D();
    return cs;
}

void Sampler::StartPixel(const Point2i &p) 
{
    currentPixel = p;
    currentPixelSampleIndex = samplingPlanner->SamplesOfPreviousIterations(currentPixel);

    //for (int i = 0; i < currentPixelSampleIndex; i++) GetCameraSample(currentPixel);

    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
}

bool Sampler::StartNextSample() 
{
    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
    currentPixelSampleIndex += 1;
    int samplesNeeded = samplingPlanner->SamplesOfPreviousIterations(currentPixel) + samplingPlanner->PlannedSamplesForThisIteration(currentPixel);
    return currentPixelSampleIndex < samplesNeeded;
}

void Sampler::AddSamplingPlanner(std::string name)
{
    if (name == "nonadaptive") samplingPlanner = std::shared_ptr<SamplingPlanner>(new NonAdaptiveSamplingPlanner());
    else if (name == "nlmeans") samplingPlanner = std::shared_ptr<SamplingPlanner>(new NLMeansSamplingPlanner());
    else LOG(FATAL) << "Sampler: AddSamplingPlanner: Given name is undefined!";
}

void Sampler::InitializeSamplingPlan(Film *film)
{
    if (samplingPlanner == nullptr) LOG(FATAL) << "Sampler does not have a samplingPlanner";

    samplingPlanner->Initialize(averagePerPixelSampleBudget, film);
    maxSamplesPerPixel = samplingPlanner->maxSamplesPerPixel;
    AdaptToSamplingPlan();
}

bool Sampler::StartNextIteration()
{
    return samplingPlanner->StartNextIteration();
}

void Sampler::UpdateSamplingPlan(Film * film)
{
    samplingPlanner->UpdateSamplingPlan(film);
}

bool Sampler::SetSampleNumber(int64_t sampleNum) {
    // Reset array offsets for next pixel sample
    array1DOffset = array2DOffset = 0;
    currentPixelSampleIndex = sampleNum;
    return currentPixelSampleIndex < maxSamplesPerPixel;
}

void Sampler::Request1DArray(int n) {
    CHECK_EQ(RoundCount(n), n);
    samples1DArraySizes.push_back(n);
    sampleArray1D.push_back(std::vector<Float>(n * maxSamplesPerPixel));
}

void Sampler::Request2DArray(int n) {
    CHECK_EQ(RoundCount(n), n);
    samples2DArraySizes.push_back(n);
    sampleArray2D.push_back(std::vector<Point2f>(n * maxSamplesPerPixel));
}

const Float *Sampler::Get1DArray(int n) {
    if (array1DOffset == sampleArray1D.size()) return nullptr;
    CHECK_EQ(samples1DArraySizes[array1DOffset], n);
    CHECK_LT(currentPixelSampleIndex, maxSamplesPerPixel);
    return &sampleArray1D[array1DOffset++][currentPixelSampleIndex * n];
}

const Point2f *Sampler::Get2DArray(int n) {
    if (array2DOffset == sampleArray2D.size()) return nullptr;
    CHECK_EQ(samples2DArraySizes[array2DOffset], n);
    CHECK_LT(currentPixelSampleIndex, maxSamplesPerPixel);
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
    CHECK_LT(currentPixelSampleIndex, maxSamplesPerPixel);
    if (current1DDimension < samples1D.size())
        return samples1D[current1DDimension++][currentPixelSampleIndex];
    else
        return rng.UniformFloat();
}

Point2f PixelSampler::Get2D() {
    ProfilePhase _(Prof::GetSample);
    CHECK_LT(currentPixelSampleIndex, maxSamplesPerPixel);
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
        samples1D.push_back(std::vector<Float>(samplingPlanner->maxSamplesPerPixel));
        samples2D.push_back(std::vector<Point2f>(samplingPlanner->maxSamplesPerPixel));
    }
}

void GlobalSampler::StartPixel(const Point2i &p) {
    ProfilePhase _(Prof::StartPixel);
    Sampler::StartPixel(p);
    dimension = 0;
    intervalSampleIndex = GetIndexForSample(samplingPlanner->SamplesOfPreviousIterations(p));
    // Compute _arrayEndDim_ for dimensions used for array samples
    arrayEndDim =
        arrayStartDim + sampleArray1D.size() + 2 * sampleArray2D.size();

    // Compute 1D array samples for _GlobalSampler_
    for (size_t i = 0; i < samples1DArraySizes.size(); ++i) {
        int nSamples = samples1DArraySizes[i] * maxSamplesPerPixel;
        for (int j = 0; j < nSamples; ++j) {
            int64_t index = GetIndexForSample(j);
            sampleArray1D[i][j] = SampleDimension(index, arrayStartDim + i);
        }
    }

    // Compute 2D array samples for _GlobalSampler_
    int dim = arrayStartDim + samples1DArraySizes.size();
    for (size_t i = 0; i < samples2DArraySizes.size(); ++i) {
        int nSamples = samples2DArraySizes[i] * maxSamplesPerPixel;
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
    return SampleDimension(intervalSampleIndex, dimension++);
}

Point2f GlobalSampler::Get2D() {
    ProfilePhase _(Prof::GetSample);
    if (dimension + 1 >= arrayStartDim && dimension < arrayEndDim)
        dimension = arrayEndDim;
    Point2f p(SampleDimension(intervalSampleIndex, dimension),
              SampleDimension(intervalSampleIndex, dimension + 1));
    dimension += 2;
    return p;
}

}  // namespace pbrt
