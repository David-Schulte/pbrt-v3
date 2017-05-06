#include "pbrt.h"
#include "AdaptiveSamplerIntegrator.h"
#include "scene.h"

namespace pbrt
{

    class AdaptiveNLMeansIntegrator : public AdaptiveSamplerIntegrator
    {
    public:

        virtual std::vector<double> CreateSampleMap() const override;

    };

}
