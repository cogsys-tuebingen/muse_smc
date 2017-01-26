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
    PluginLoader<Update>::load(nh_private_,      update_functions_);
    PluginLoader<Propagation>::load(nh_private_, prediction_);

    PluginLoader<MapProvider>::load(nh_private_,  map_providers_);
    PluginLoader<DataProvider>::load(nh_private_, data_providers_);

    //// sampling algorithms
    UniformSampling::Ptr uniform_sampling;
    NormalSampling::Ptr  normal_sampling;
    Resampling::Ptr      resampling;


    PluginLoader<UniformSampling,  std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr>::load(nh_private_, uniform_sampling, map_providers_, tf_provider_backend_);
    PluginLoader<NormalSampling,   std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr>::load(nh_private_, normal_sampling, map_providers_,  tf_provider_backend_);
    PluginLoader<Resampling, UniformSampling::Ptr>::load(nh_private_, resampling, uniform_sampling);

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

