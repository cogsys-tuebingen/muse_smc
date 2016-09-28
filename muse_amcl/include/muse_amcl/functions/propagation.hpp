#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

namespace muse_amcl {
struct Propagation {
    typedef std::shared_ptr<Propagation> Ptr;

    virtual void apply(std::vector<tf::Pose> &poses,
                       const std::vector<double> &weights) = 0;
};
}
