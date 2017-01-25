#ifndef MULTINOMIAL_H
#define MULTINOMIAL_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Multinomial : public Resampling
{
public:
    Multinomial() = default;

protected:
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // MULTINOMIAL_H
