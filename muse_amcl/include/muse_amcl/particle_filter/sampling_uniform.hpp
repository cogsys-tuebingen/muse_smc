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
class UniformSampling {
public:
    typedef std::shared_ptr<UniformSampling> Ptr;

    using MapProviders = std::map<std::string, MapProvider::Ptr>;

    UniformSampling()
    {
    }

    virtual ~UniformSampling()
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

    inline void setup(const std::string                             &name,
                      ros::NodeHandle                               &nh_private,
                      const std::map<std::string, MapProvider::Ptr> &map_providers,
                      const TFProvider::Ptr                         &tf_provider)
    {
        double sampling_timeout;
        double tf_timeout;
        name_              = name;
        sample_size_       = nh_private.param(parameter("sample_size"), 500);
        sampling_timeout   = nh_private.param(parameter("timeout"), 10.0);
        tf_timeout         = nh_private.param(parameter("tf_timeout"), 0.1);
        sampling_timeout_  = ros::Duration(sampling_timeout);
        tf_timeout_        = ros::Duration(tf_timeout);
        tf_provider_       = tf_provider;

        doSetup(nh_private);
        doSetupMapProviders(nh_private, map_providers);
    }

    /**
     * @brief Has to be called before generation to capture the worlds state
     *        at the current time. Afterwards resampling may be called.
     * @param frame - frame poses should be generated in
     */
    virtual inline void update(const std::string &frame) = 0;
    /**
     * @brief Build a uniformely distributed particle set using maps that should
     *        be used for pose generation.
     */
    virtual void apply(ParticleSet &particle_set) = 0;
    /**
     * @brief Generate a single particle from current state.
     * @param particle
     */
    virtual void apply(Particle &particle) = 0;

protected:
    std::string                   name_;
    std::size_t                   sample_size_;
    std::vector<MapProvider::Ptr> map_providers_;
    ros::Duration                 sampling_timeout_;
    ros::Duration                 tf_timeout_;
    TFProvider::Ptr               tf_provider_;

    virtual void doSetup(ros::NodeHandle &nh_private) = 0;
    virtual void doSetupMapProviders(ros::NodeHandle &nh_private,
                                     const MapProviders &map_providers)
    {
        std::vector<std::string> map_provider_ids;
        nh_private.getParam(parameter("maps"), map_provider_ids);

        for(auto m : map_provider_ids) {
            map_providers_.emplace_back(map_providers.at(m));
        }
    }

    inline std::string parameter (const std::string &name)
    {
        return name_ + "/" + name;
    }
};
}



#endif // POSE_GENERATION_UNIFORM_HPP
