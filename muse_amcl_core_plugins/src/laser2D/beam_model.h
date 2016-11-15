#pragma once

#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class BeamModel : public Update
{
public:
    BeamModel();

    virtual double apply(const Data::ConstPtr &data,
                         const Map::ConstPtr &map,
                         ParticleSet::WeightIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}

