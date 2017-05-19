#ifndef RESIDUAL_H
#define RESIDUAL_H

#include <muse_mcl/particle_filter/resampling.hpp>

namespace muse_mcl {
class Residual : public Resampling
{
public:
    Residual() = default;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;
};
}

#endif // RESIDUAL_H
