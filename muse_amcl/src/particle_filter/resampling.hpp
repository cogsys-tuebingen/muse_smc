#pragma once
namespace muse_amcl {
namespace particle_filter {
#include <memory>
#include <vector>
#include <tf/tf.h>

struct Resampling {
    typedef std::shared_ptr<Resampling> Ptr;

    virtual void apply(std::vector<tf::Pose> &poses,
                       std::vector<double> &weights) = 0;
};
}
}
