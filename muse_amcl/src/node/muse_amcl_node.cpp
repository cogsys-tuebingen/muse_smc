#include "muse_amcl_node.h"

using namespace muse_amcl;

MuseAMCLNode::MuseAMCLNode() :
    nh_private_("~")
{

}

MuseAMCLNode::~MuseAMCLNode()
{
}

void MuseAMCLNode::start()
{

}

bool MuseAMCLNode::requestGlobalInitialization(const muse_amcl::GlobalInitializationRequest &req,
                                               muse_amcl::GlobalInitializationResponse &res)
{

}

bool MuseAMCLNode::requestPoseInitialization(const muse_amcl::PoseInitializationRequest &req,
                                             muse_amcl::PoseInitializationResponse &res)
{

}

void MuseAMCLNode::setup()
{
    /// load all plugins
    PluginLoader<UpdateModel, TFProvider::Ptr, ros::NodeHandle&>::load(update_models_, tf_provider_backend_, nh_private_);
    PluginLoader<PredictionModel, TFProvider::Ptr, ros::NodeHandle&>::load(prediction_model_, tf_provider_backend_, nh_private_);

    PluginLoader<MapProvider, ros::NodeHandle&>::load(map_providers_, nh_private_);
    PluginLoader<DataProvider, ros::NodeHandle&>::load(data_providers_, nh_private_);

    //// sampling algorithms
    UniformSampling::Ptr uniform_sampling;
    NormalSampling::Ptr  normal_sampling;
    Resampling::Ptr      resampling;


    PluginLoader<UniformSampling,  std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr, ros::NodeHandle&>::load(uniform_sampling, map_providers_, tf_provider_backend_, nh_private_);
    PluginLoader<NormalSampling,   std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr, ros::NodeHandle&>::load(normal_sampling, map_providers_,  tf_provider_backend_, nh_private_);
    PluginLoader<Resampling, UniformSampling::Ptr, ros::NodeHandle&>::load(resampling, uniform_sampling, nh_private_);

    particle_filter_.setNormalsampling(normal_sampling);
    particle_filter_.setUniformSampling(uniform_sampling);
    particle_filter_.setResampling(resampling);

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl");
    MuseAMCLNode node;


    return 0;
}

