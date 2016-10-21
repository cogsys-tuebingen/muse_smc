#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>

#include <muse_amcl/pf/particle_set.hpp>

namespace muse_amcl {
struct Update {
    typedef std::shared_ptr<Update> Ptr;

    Update() :
        nh_private("~")
    {
    }

    virtual void setup(const std::string &name) = 0;
    virtual double apply(ParticleSet::WeightIterator set) = 0;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

};
}
