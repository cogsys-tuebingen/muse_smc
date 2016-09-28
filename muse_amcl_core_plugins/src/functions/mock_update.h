#pragma once

#include <muse_amcl/functions/update.hpp>

namespace muse_amcl {
class MockUpdate : public Update
{
public:
    MockUpdate();

    virtual void apply(const std::vector<tf::Pose> &poses,
                       std::vector<double> &weights) override;

};
}
