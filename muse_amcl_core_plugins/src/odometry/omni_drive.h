#pragma once

#include <muse_amcl/plugins/types/propagation.hpp>

namespace muse_amcl {
class OmniDrive : public Propagation
{
public:
    OmniDrive();

    virtual void apply(Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;
};
}
