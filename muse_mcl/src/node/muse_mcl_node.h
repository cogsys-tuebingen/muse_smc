#ifndef MUSE_MCL_NODE_H
#define MUSE_MCL_NODE_H

#include <muse_mcl/particle_filter/particle_filter.hpp>

#include <muse_mcl/GlobalInitialization.h>
#include <muse_mcl/PoseInitialization.h>

#include <muse_mcl/tf/tf_provider.hpp>

#include <muse_mcl/providers/provider_data.hpp>
#include <muse_mcl/providers/provider_map.hpp>

#include <muse_mcl/update/update_forwarder.hpp>
#include <muse_mcl/prediction/prediction_forwarder.hpp>
#include <muse_mcl/particle_filter/particle_filter.hpp>

#include <muse_mcl/plugins/plugin_loader.hpp>
#include <muse_mcl/plugins/plugin_factory.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf/time_cache.h>

#include <ros/ros.h>

namespace muse_mcl {
class MuseMCLNode
{
public:
    MuseMCLNode();
    virtual ~MuseMCLNode();

    bool setup();

    void start();

    bool requestGlobalInitialization(muse_mcl::GlobalInitialization::Request &req,
                                     muse_mcl::GlobalInitialization::Response &res);

    bool requestPoseInitialization(muse_mcl::PoseInitialization::Request &req,
                                   muse_mcl::PoseInitialization::Response &res);

    void poseInitialization(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

private:
    using MapProviders  = std::map<std::string, ProviderMap::Ptr>;
    using DataProviders = std::map<std::string, ProviderData::Ptr>;
    using UpdateModels  = std::map<std::string, ModelUpdate::Ptr>;

    ros::NodeHandle             nh_private_;
    ros::NodeHandle             nh_public_;
    ros::ServiceServer          initialization_service_global_;
    ros::ServiceServer          initialization_service_pose_;
    ros::Subscriber             initialization_subscriber_pose_;

    //// data providers
    TFProvider::Ptr             tf_provider_frontend_;  /// for data providers and data conversion
    TFProvider::Ptr             tf_provider_backend_;   /// for the backend (the particle filter and the sensor updates)
    MapProviders                map_providers_;
    DataProviders               data_providers_;

    ParticleFilter::Ptr         particle_filter_;

    //// prediction & update
    UpdateModels                update_models_;
    ModelPrediction::Ptr        prediction_model_;

    /// sampling & resampling
    SamplingUniform::Ptr        uniform_sampling_;
    SamplingNormal::Ptr         normal_sampling_;
    Resampling::Ptr             resampling_;


    UpdateForwarder::Ptr        update_forwarder_;
    PredictionForwarder::Ptr    predicition_forwarder_;

    void checkPoseInitialization();

};
}

#endif /* MUSE_MCL_NODE_H */
