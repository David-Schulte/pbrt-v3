// core/integrator.h*
#include "pbrt.h"
#include "integrator.h"
#include "primitive.h"
#include "spectrum.h"
#include "light.h"
#include "reflection.h"
#include "sampler.h"
#include "material.h"

namespace pbrt
{

class AdaptiveSamplerIntegrator : public Integrator
{
public:
    // SamplerIntegrator Public Methods
    AdaptiveSamplerIntegrator(std::shared_ptr<const Camera> camera,
                              std::shared_ptr<Sampler> sampler,
                              const Bounds2i &pixelBounds)
                              : camera(camera), sampler(sampler), pixelBounds(pixelBounds) {}

    virtual void Preprocess(const Scene &scene, Sampler &sampler) {}

    void Render(const Scene &scene);

    virtual void AdaptiveIteration(Point2i currentTile) const = 0;

    virtual std::vector<double> CreateSampleMap() const = 0;

    virtual Spectrum Li(const RayDifferential &ray, const Scene &scene,
                        Sampler &sampler, MemoryArena &arena,
                        int depth = 0) const = 0;

    Spectrum SpecularReflect(const RayDifferential &ray,
                             const SurfaceInteraction &isect,
                             const Scene &scene, Sampler &sampler,
                             MemoryArena &arena, int depth) const;

    Spectrum SpecularTransmit(const RayDifferential &ray,
                              const SurfaceInteraction &isect,
                              const Scene &scene, Sampler &sampler,
                              MemoryArena &arena, int depth) const;

protected:
    std::shared_ptr<const Camera> camera;

    void CheckRadiance(Spectrum &radiance, const Point2i pixel, const std::unique_ptr<Sampler> &sampler);
    Bounds2i BoundsForTile(const Point2i tile);
    void RenderTile(const Scene &scene, const Point2i tile);

private:
    std::shared_ptr<Sampler> sampler;
    const Bounds2i pixelBounds;
    Bounds2i sampleBounds;
    Vector2i sampleExtent;
    const int tileSize = 16;
    Point2i nTiles;
};

}  // namespace pbrt