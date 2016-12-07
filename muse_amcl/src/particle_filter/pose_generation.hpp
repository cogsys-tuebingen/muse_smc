#ifndef POSE_GENERATION_HPP
#define POSE_GENERATION_HPP

#include <memory>
#include <vector>
#include <tf/tf.h>

namespace muse_amcl {
namespace particle_filter {
struct PoseGeneration {
    typedef std::shared_ptr<PoseGeneration> Ptr;

    virtual void apply(std::vector<tf::Pose> &poses,
                       std::vector<double> &weights) = 0;

    virtual void apply(std::vector<tf::Pose> &poses,
                       std::vector<double> &weights,
                       tf::Pose &prior) = 0;

};
}
}

#endif /* POSE_GENERATION_HPP */
