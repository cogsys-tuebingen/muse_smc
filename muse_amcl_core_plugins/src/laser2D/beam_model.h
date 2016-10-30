#pragma once

#include <muse_amcl/plugins/update.hpp>

namespace muse_amcl {
class BeamModel : public Update
{
public:
    BeamModel();

    virtual double apply(ParticleSet::WeightIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}

