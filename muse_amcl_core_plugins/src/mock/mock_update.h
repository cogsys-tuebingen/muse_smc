#pragma once

#include <muse_amcl/plugins/types/update.hpp>

namespace muse_amcl {
class MockUpdate : public Update
{
public:
    typedef std::shared_ptr<MockUpdate> Ptr;

    MockUpdate();

    virtual double apply(Data::ConstPtr &data,
                         ParticleSet::WeightIterator set) override;

    virtual void setMap(Map::ConstPtr &map) override;

    double  first_parameter;
    int     second_parameter;

protected:
    virtual void loadParameters(ros::NodeHandle &nh_private) override;


};
}
