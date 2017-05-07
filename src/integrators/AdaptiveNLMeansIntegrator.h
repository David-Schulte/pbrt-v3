#include "pbrt.h"
#include "AdaptiveSamplerIntegrator.h"
#include "scene.h"

namespace pbrt
{

    class AdaptiveNLMeansIntegrator : public AdaptiveSamplerIntegrator
    {
    public:
        AdaptiveNLMeansIntegrator(std::shared_ptr<const Camera> camera, std::shared_ptr<Sampler> sampler, const Bounds2i &pixelBounds) 
            : AdaptiveSamplerIntegrator(camera, sampler, pixelBounds) {}

        virtual void Render(const Scene &scene) override;
        virtual void UpdateSampler() override;

    protected:
        void UniformSampling();
    };

}
