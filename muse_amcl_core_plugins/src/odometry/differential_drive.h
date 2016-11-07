#pragma once

#include <muse_amcl/plugins/types/propagation.hpp>

namespace muse_amcl {
class DifferentialDrive : public Propagation
{
public:
    DifferentialDrive();

    virtual void apply(Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;

};
}

