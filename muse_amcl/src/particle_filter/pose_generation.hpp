#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

namespace muse_amcl {
namespace particle_filter {
struct PoseGeneration {
    typedef std::shared_ptr<Update> Ptr;

    virtual void apply(std::vector<tf::Pose> &poses,
                       std::vector<double> &weights) = 0;

    virtual void apply(std::vector<tf::Pose> &poses,
                       std::vector<double> &weights,
                       tf::Pose &prior) = 0;

};
}
}
