#ifndef MAJORMAPAA2D_H
#define MAJORMAPAA2D_H

#include <muse_amcl/particle_filter/sampling_uniform.hpp>

namespace muse_amcl {
class UniformPrimaryMap2D : public UniformSampling
{
public:
    UniformPrimaryMap2D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:
    MapProvider::Ptr primary_map_provider_;
    int              random_seed_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doSetupMapProviders(ros::NodeHandle &nh_private,
                                     const MapProviders &map_providers) override;

};
}

#endif // MAJORMAPAA2D_H
