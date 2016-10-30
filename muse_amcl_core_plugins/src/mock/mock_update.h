#pragma once

#include <muse_amcl/plugins/update.hpp>

namespace muse_amcl {
class MockUpdate : public Update
{
public:
    MockUpdate();

    virtual double apply(ParticleSet::WeightIterator set) override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh) override;


};
}
