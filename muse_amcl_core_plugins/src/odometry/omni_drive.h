#pragma once

#include <muse_amcl/plugins/propagation.hpp>

namespace muse_amcl {
class OmniDrive : public Propagation
{
public:
    OmniDrive();

    virtual void apply(ParticleSet::PoseIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;
};
}
