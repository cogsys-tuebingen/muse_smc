#ifndef WHEELOFFORTUNE_H
#define WHEELOFFORTUNE_H

#include <muse_mcl/plugins/types/resampling.hpp>

namespace muse_mcl {
class WheelOfFortune : public Resampling
{
public:
    WheelOfFortune() = default;

protected:
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;
};
}

#endif // WHEELOFFORTUNE_H
