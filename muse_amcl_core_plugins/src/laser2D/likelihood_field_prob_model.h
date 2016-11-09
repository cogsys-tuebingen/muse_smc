#pragma once

#include <muse_amcl/plugins/types/update.hpp>

namespace muse_amcl {
class LikelihoodFieldProbModel : public Update
{
public:
    LikelihoodFieldProbModel();

    virtual double apply(Data::ConstPtr &data,
                         Map::ConstPtr &map,
                         ParticleSet::WeightIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;

};
}

