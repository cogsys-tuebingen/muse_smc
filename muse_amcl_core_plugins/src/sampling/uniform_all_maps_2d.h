#ifndef ENCLOSINGAA2D_H
#define ENCLOSINGAA2D_H

#include <muse_amcl/particle_filter/sampling_uniform.hpp>

namespace muse_amcl {
class UniformAllMaps2D : public UniformSampling
{
public:
    UniformAllMaps2D();

    virtual void apply(ParticleSet &particle_set) override;

protected:
    int random_seed_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // ENCLOSINGAA2D_H
