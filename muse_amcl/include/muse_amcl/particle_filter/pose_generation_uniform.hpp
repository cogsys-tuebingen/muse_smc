#ifndef POSE_GENERATION_UNIFORM_HPP
#define POSE_GENERATION_UNIFORM_HPP

#include <memory>
#include <vector>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>

#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/data_sources/tf_provider.hpp>
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

    inline std::string getName() const
    {
        return name_;
    }

    void setup(const std::string &name,
               ros::NodeHandle &nh_private)
    {
        double sampling_timeout;
        double tf_timeout;
        name_              = name;
        sample_size_       = nh_private.param(parameter("sample_size"), 500);
        sampling_timeout   = nh_private.param(parameter("timeout"), 10.0);
        tf_timeout         = nh_private.param(parameter("tf_timeout"), 0.1);
        nh_private.getParam(parameter("/maps"), map_provider_ids_);
        sampling_timeout_  = ros::Duration(sampling_timeout);
        tf_timeout_        = ros::Duration(tf_timeout);

        doSetup(nh_private);
    }


    void setTF(const TFProvider::Ptr &tf)
    {
        tf_ = tf;
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
    std::size_t                   sample_size_;
    std::vector<std::string>      map_provider_ids_;
    std::vector<MapProvider::Ptr> maps_providers_;
    ros::Duration                 sampling_timeout_;
    ros::Duration                 tf_timeout_;
    TFProvider::Ptr               tf_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;

    std::string parameter (const std::string &name)
    {
        return name_ + "/" + name;
    }
};
}



#endif // POSE_GENERATION_UNIFORM_HPP
