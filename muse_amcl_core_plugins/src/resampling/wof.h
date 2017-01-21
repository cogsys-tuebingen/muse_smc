#ifndef WHEELOFFORTUNE_H
#define WHEELOFFORTUNE_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class WheelOfFortune : public Resampling
{
public:
    WheelOfFortune() = default;

    virtual void apply(ParticleSet &p_t_1) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // WHEELOFFORTUNE_H
