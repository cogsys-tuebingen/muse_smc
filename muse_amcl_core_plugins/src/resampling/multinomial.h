#ifndef MULTINOMIAL_H
#define MULTINOMIAL_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_mcl {
class Multinomial : public Resampling
{
public:
    Multinomial() = default;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;
};
}

#endif // MULTINOMIAL_H
