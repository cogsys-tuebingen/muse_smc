#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
struct Propagation {
    typedef std::shared_ptr<Propagation> Ptr;

    Propagation() :
        nh_private("~")
    {
    }

    virtual void setup(const std::string &name) = 0;
    virtual void apply(ParticleSet::PoseIterator set) = 0;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;


};
}
