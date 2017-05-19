#ifndef SYSTEMATIC_H
#define SYSTEMATIC_H

#include <muse_mcl/particle_filter/resampling.hpp>

namespace muse_mcl {
class Systematic : public Resampling
{
public:
    Systematic() = default;

protected:
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;
};
}
#endif // SYSTEMATIC_H
