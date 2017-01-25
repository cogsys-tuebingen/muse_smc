#ifndef RESIDUAL_H
#define RESIDUAL_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Residual : public Resampling
{
public:
    Residual() = default;

protected:
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // RESIDUAL_H
