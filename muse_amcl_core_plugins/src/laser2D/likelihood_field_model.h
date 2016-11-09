#pragma once

#include <muse_amcl/plugins/types/update.hpp>

namespace muse_amcl {
class LikelihoodFieldModel : public Update
{
public:
    LikelihoodFieldModel();

    virtual double apply(Data::ConstPtr &data,
                         ParticleSet::WeightIterator set) override;

    virtual void setMap(Map::ConstPtr &map) override;
    virtual bool hasMap() const override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;

};
}
