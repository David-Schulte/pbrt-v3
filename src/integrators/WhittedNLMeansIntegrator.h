#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_NLMEANS_WHITTED_H
#define PBRT_INTEGRATORS_NLMEANS_WHITTED_H

#include "pbrt.h"
#include "AdaptiveNLMeansIntegrator.h"
#include "scene.h"

namespace pbrt 
{

    class WhittedNLMeansIntegrator : public AdaptiveNLMeansIntegrator 
    {
    public:
         WhittedNLMeansIntegrator(int maxDepth, std::shared_ptr<const Camera> camera, std::shared_ptr<Sampler> sampler, const Bounds2i &pixelBounds)
             : AdaptiveNLMeansIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

        Spectrum Li(const RayDifferential &ray, const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const;

    private:
        const int maxDepth;
    };

    WhittedNLMeansIntegrator *CreateWhittedNLMeansIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler, 
                                                             std::shared_ptr<const Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_NLMEANS_WHITTED_H
