#pragma once

#include <memory>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include "../particle_filter/particle_set.hpp"
#include "../data_sources/map_provider.hpp"
#include "../math/random.hpp"

// #include <muse_amcl/particle_filter/map_manager.hpp>
// #include <muse_amcl/math/pose.hpp>
// #include <muse_amcl/math/random.hpp>
#include <muse_amcl/math/covariance.hpp>

namespace muse_amcl {
class PoseGeneration {
public:
    typedef std::shared_ptr<PoseGeneration> Ptr;

    /**
     * @brief Generate a multivariate Gaussian distributed particle set with
     *        a mean given by a desired initialization pose and the covariance
     *        matrix.
     * @param pose          - 6 dimensional pose vector (x,y,z,roll,pitch,yaw)
     * @param covariance    - 6 dimensnioal covariance matrix
     */
    virtual void normal(const math::Pose       &pose,
                        const math::Covariance &covariance,
                        ParticleSet            &particle_set) = 0;

    /**
     * @brief Build a uniformely distributed particle set using maps that should
     *        be used for pose generation.
     */
    virtual void uniform(ParticleSet &particle_set) = 0;

    void setup(const std::map<std::string, MapProvider::Ptr>  &map_providers,
               ros::NodeHandle &nh_private)
    {
        frame_id_    = nh_private.param("particle_filter/frame_id", "/world");
        sample_size_ = nh_private.param("particle_filter/pose_generation/sample_size", 500);

        std::vector<std::string> map_keys;
        nh_private.getParam("particle_filter/prose_generation/maps", map_keys);
        for(std::string &k : map_keys) {
            map_providers_.emplace_back(map_providers.at(k));
        }
    }

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string                     frame_id_;
    std::size_t                     sample_size_;
    std::vector<MapProvider::Ptr>   map_providers_;

};
}
