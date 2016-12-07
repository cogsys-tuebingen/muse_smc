#ifndef MULTINOMIAL_H
#define MULTINOMIAL_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Multinomial : public Resampling
{
public:
    Multinomial() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // MULTINOMIAL_H
