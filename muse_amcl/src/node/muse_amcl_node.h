#ifndef MUSE_AMCL_NODE_H
#define MUSE_AMCL_NODE_H

#include <muse_amcl/particle_filter/particle_filter.hpp>

#include <muse_amcl/GlobalInitialization.h>
#include <muse_amcl/PoseInitialization.h>

#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl/data_sources/data_provider.hpp>
#include <muse_amcl/data_sources/tf_provider.hpp>

#include <muse_amcl/particle_filter/update_forwarder.hpp>
#include <muse_amcl/particle_filter/prediction_forwarder.hpp>
#include <muse_amcl/particle_filter/particle_filter.hpp>

#include <muse_amcl/plugins/plugin_loader.hpp>
#include <muse_amcl/plugins/plugin_factory.hpp>

#include <ros/ros.h>


namespace muse_amcl {
class MuseAMCLNode
{
public:
    MuseAMCLNode();
    virtual ~MuseAMCLNode();

    void start();

    bool requestGlobalInitialization(const muse_amcl::GlobalInitializationRequest &req,
                                     muse_amcl::GlobalInitializationResponse &res);

    bool requestPoseInitialization(const muse_amcl::PoseInitializationRequest &req,
                                   muse_amcl::PoseInitializationResponse &res);

private:
    using MapProviders  = std::map<std::string, MapProvider::Ptr>;
    using DataProviders = std::map<std::string, DataProvider::Ptr>;
    using UpdateModels  = std::map<std::string, UpdateModel::Ptr>;

    ros::NodeHandle             nh_private_;
    ros::NodeHandle             nh_public_;

    //// data providers
    TFProvider::Ptr             tf_provider_frontend_;  /// for data providers and data conversion
    TFProvider::Ptr             tf_provider_backend_;   /// for the backend (the particle filter and the sensor updates)
    MapProviders                map_providers_;
    DataProviders               data_providers_;

    ParticleFilter::Ptr         particle_filter_;

    //// prediction & update
    UpdateModels                update_models_;
    PredictionModel::Ptr        prediction_model_;

    /// sampling & resampling
    UniformSampling::Ptr        uniform_sampling_;
    NormalSampling::Ptr         normal_sampling_;
    Resampling::Ptr             resampling_;


    UpdateForwarder::Ptr        update_forwarder_;
    PredictionForwarder::Ptr    predicition_forwarder_;

    /// read all ros related stuff and initalize plugins
    void setup();
};
}

#endif /* MUSE_AMCL_NODE_H */
