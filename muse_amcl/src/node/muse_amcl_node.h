#ifndef MUSE_AMCL_NODE_H
#define MUSE_AMCL_NODE_H

#include "../../include/muse_amcl/particle_filter/particle_filter.hpp"

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
    ros::NodeHandle                          nh_private_;
    ros::NodeHandle                          nh_public_;

    //// data providers
    TFProvider::Ptr                          tf_provider_frontend_;  /// for data providers and data conversion
    TFProvider::Ptr                          tf_provider_backend_;   /// for the backend (the particle filter and the sensor updates)
    std::map<std::string, MapProvider::Ptr>  map_providers_;
    std::map<std::string, DataProvider::Ptr> data_providers_;

    ParticleFilter                           particle_filter_;

    //// prediction & update
    std::map<std::string, UpdateModel::Ptr>  update_models_;
    PredictionModel::Ptr                     prediction_model_;

    UpdateForwarder::Ptr                       update_manager_;
    PredictionForwarder::Ptr                  predicition_manager_;

    /// read all ros related stuff and initalize plugins
    void setup();
};
}

#endif /* MUSE_AMCL_NODE_H */
