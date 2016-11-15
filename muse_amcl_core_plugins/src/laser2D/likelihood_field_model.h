#pragma once

#include <muse_amcl/particle_filter/update.hpp>

namespace muse_amcl {
class LikelihoodFieldModel : public Update
{
public:
    LikelihoodFieldModel();

    virtual double apply(Data::ConstPtr &data,
                         Map::ConstPtr &map,
                         ParticleSet::WeightIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;

};
}
