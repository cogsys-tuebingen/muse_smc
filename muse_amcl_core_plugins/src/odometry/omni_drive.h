#pragma once

#include <muse_amcl/particle_filter/propagation.hpp>

namespace muse_amcl {
class OmniDrive : public Propagation
{
public:
    OmniDrive();

    virtual void apply(const Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;
};
}
