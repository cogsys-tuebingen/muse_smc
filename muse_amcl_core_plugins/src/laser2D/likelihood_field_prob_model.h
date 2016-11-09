#pragma once

#include <muse_amcl/plugins/types/update.hpp>

namespace muse_amcl {
class LikelihoodFieldProbModel : public Update
{
public:
    LikelihoodFieldProbModel();

    virtual double apply(Data::ConstPtr &data,
                         ParticleSet::WeightIterator set) override;

    virtual void setMap(Map::ConstPtr &map) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;

};
}

