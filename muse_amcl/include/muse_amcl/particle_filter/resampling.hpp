#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include "../particle_filter/particle_set.hpp"

namespace muse_amcl {
class Resampling {
public:
    typedef std::shared_ptr<Resampling> Ptr;

    Resampling(ParticleSet &particle_set) :
        particle_set_(particle_set)
    {
    }

    void resample() = 0;

    void setup(ros::NodeHandle &nh_private) = 0;
private:
    ParticleSet &particle_set_;
};
}
