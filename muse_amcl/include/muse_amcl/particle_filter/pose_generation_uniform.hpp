#ifndef POSE_GENERATION_UNIFORM_HPP
#define POSE_GENERATION_UNIFORM_HPP

#include <memory>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/math/covariance.hpp>

namespace muse_amcl {
class UniformPoseGeneration {
public:
    typedef std::shared_ptr<UniformPoseGeneration> Ptr;

    UniformPoseGeneration()
    {
    }

    virtual ~UniformPoseGeneration()
    {
    }

    inline const static std::string Type()
    {
        return "muse_amcl::UniformPoseGeneration";
    }

    inline std::string name() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle &nh_private)
    {
        name_              = name;
        frame_id_          = nh_private.param<std::string>("particle_filter/frame_id", "/world");
        sample_size_       = nh_private.param("particle_filter/pose_generation/sample_size", 500);
        nh_private.getParam("maps", map_provider_ids_);
        doSetup(nh_private);
    }

    void setMapProviders(const std::map<std::string, MapProvider::Ptr> &map_providers)
    {
        for(auto m : map_provider_ids_) {
            maps_providers_.emplace_back(map_providers.at(m));
        }
    }


    /**
     * @brief Build a uniformely distributed particle set using maps that should
     *        be used for pose generation.
     */
    virtual void apply(ParticleSet &particle_set) = 0;

protected:
    std::string                   name_;
    std::string                   frame_id_;
    std::size_t                   sample_size_;
    std::vector<std::string>      map_provider_ids_;
    std::vector<MapProvider::Ptr> maps_providers_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string param (const std::string &name)
    {
        return name_ + "/" + name;
    }
};
}



#endif // POSE_GENERATION_UNIFORM_HPP
