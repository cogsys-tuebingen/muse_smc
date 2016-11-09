#pragma once

#include <muse_amcl/plugins/types/update.hpp>

namespace muse_amcl {
class BeamModel : public Update
{
public:
    BeamModel();

    virtual double apply(Data::ConstPtr &data,
                         ParticleSet::WeightIterator set) override;

    virtual void setMap(Map::ConstPtr &map) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}

