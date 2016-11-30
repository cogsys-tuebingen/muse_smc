#ifndef POSE_GENERATION_NORMAL_HPP
#define POSE_GENERATION_NORMAL_HPP

#include <memory>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/math/covariance.hpp>

namespace muse_amcl {
class NormalPoseGeneration {
public:
    typedef std::shared_ptr<NormalPoseGeneration> Ptr;

    NormalPoseGeneration()
    {
    }

    virtual ~NormalPoseGeneration()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::NormalPoseGeneration";
    }

    inline std::string name() const
    {
        return name_;
    }

    void setup(const std::string &name,
               const std::map<std::string, MapProvider::Ptr>  &map_providers,
               ros::NodeHandle &nh_private)
    {
        name_              = name;
        frame_id_          = nh_private.param<std::string>("particle_filter/frame_id", "/world");
        sample_size_       = nh_private.param("particle_filter/pose_generation/sample_size", 500);
        doSetup(map_providers, nh_private);
    }

    /**
     * @brief Generate a multivariate Gaussian distributed particle set with
     *        a mean given by a desired initialization pose and the covariance
     *        matrix.
     * @param pose          - 6 dimensional pose vector (x,y,z,roll,pitch,yaw)
     * @param covariance    - 6 dimensnioal covariance matrix
     */
    virtual void apply(const math::Pose       &pose,
                       const math::Covariance &covariance,
                       ParticleSet            &particle_set) = 0;

protected:
    std::string name_;

    virtual void doSetup(const std::map<std::string, MapProvider::Ptr>  &map_providers,
                         ros::NodeHandle &nh_private) = 0;

    std::string  frame_id_;
    std::size_t  sample_size_;

    std::string param (const std::string &name)
    {
        return name_ + "/" + name;
    }

};
}
#endif // POSE_GENERATION_NORMAL_HPP
