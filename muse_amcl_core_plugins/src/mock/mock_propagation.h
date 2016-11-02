#pragma once

#include <muse_amcl/plugins/propagation.hpp>

namespace muse_amcl {
class MockPropagation : public Propagation
{
public:
    typedef std::shared_ptr<MockPropagation> Ptr;

    MockPropagation();

    virtual void apply(ParticleSet::PoseIterator set) override;


    double  first_parameter;
    int     second_parameter;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}

