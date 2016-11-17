#pragma once

#include <memory>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include "../particle_filter/particle_set.hpp"
#include "../data_sources/map_provider.hpp"
#include "../math/random.hpp"

namespace muse_amcl {
class ParticleGeneration {
public:
    typedef std::shared_ptr<ParticleGeneration> Ptr;

    ParticleGeneration(ParticleSet &particle_set) :
        particle_set_(particle_set)
    {
    }

    void gaussian() = 0;

    void uniform() = 0;

    void setup(const std::map<std::string, MapProvider::Ptr>  &map_providers,
               ros::NodeHandle &nh_private)
    {
        /// build a list of maps that should be included
    }

private:
    ParticleSet                  &particle_set_;
    std::vector<MapProvider::Ptr> maps_;
};
}
