#ifndef RESIDUAL_H
#define RESIDUAL_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Residual : public Resampling
{
public:
    Residual() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // RESIDUAL_H
