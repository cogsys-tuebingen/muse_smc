#pragma once

#include <muse_amcl/plugins/types/propagation.hpp>

namespace muse_amcl {
class MockPropagation : public Propagation
{
public:
    typedef std::shared_ptr<MockPropagation> Ptr;

    MockPropagation();

    virtual void apply(Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) override;


    double  first_parameter;
    int     second_parameter;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}

