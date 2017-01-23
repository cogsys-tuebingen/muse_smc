#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include <muse_amcl/particle_filter/propagation.hpp>

namespace muse_amcl {
class OmniDrive : public Propagation
{
public:
    OmniDrive();

    virtual void apply(const Data::ConstPtr &data,
                       ParticleSet::Poses set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif /* OMNI_DRIVE_H */
