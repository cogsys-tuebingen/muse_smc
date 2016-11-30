#ifndef SYSTEMATIC_H
#define SYSTEMATIC_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Systematic : public Resampling
{
public:
    Systematic() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}
#endif // SYSTEMATIC_H
