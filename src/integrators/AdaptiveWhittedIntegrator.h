#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#ifndef PBRT_INTEGRATORS_ADAPTIVE_WHITTED_H
#define PBRT_INTEGRATORS_ADAPTIVE_WHITTED_H

// integrators/AdaptiveWhittedIntegrator.h*
#include "pbrt.h"
#include "AdaptiveSamplerIntegrator.h"
#include "scene.h"

namespace pbrt 
{

    // WhittedIntegrator Declarations
    class AdaptiveWhittedIntegrator : public AdaptiveSamplerIntegrator 
    {
    public:
        // WhittedIntegrator Public Methods
        AdaptiveWhittedIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
                                  std::shared_ptr<Sampler> sampler, const Bounds2i &pixelBounds)
            : AdaptiveSamplerIntegrator(camera, sampler, pixelBounds), maxDepth(maxDepth) {}

        Spectrum Li(const RayDifferential &ray, const Scene &scene,
                    Sampler &sampler, MemoryArena &arena, int depth) const;

    private:
        const int maxDepth;
    };

    AdaptiveWhittedIntegrator *CreateAdaptiveWhittedIntegrator(const ParamSet &params, std::shared_ptr<Sampler> sampler,
                                                               std::shared_ptr<const Camera> camera);

}  // namespace pbrt

#endif  // PBRT_INTEGRATORS_ADAPTIVE_WHITTED_H
