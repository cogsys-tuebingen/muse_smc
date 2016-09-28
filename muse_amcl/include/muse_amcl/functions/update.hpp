#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

namespace muse_amcl {
namespace particle_filter {
struct Update {
    typedef std::shared_ptr<Update> Ptr;

    virtual void apply(const std::vector<tf::Pose> &poses,
                       std::vector<double> &weights) = 0;
};
}
}
