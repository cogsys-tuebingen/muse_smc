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
    PluginLoader loader(nh_private_);

    loader.load<UpdateModel, TFProvider::Ptr, ros::NodeHandle&>(update_models_, tf_provider_backend_, nh_private_);
    loader.load<PredictionModel, TFProvider::Ptr, ros::NodeHandle&>(prediction_model_, tf_provider_backend_, nh_private_);

    loader.load<MapProvider, ros::NodeHandle&>(map_providers_, nh_private_);
    loader.load<DataProvider,TFProvider::Ptr, ros::NodeHandle&>(data_providers_, tf_provider_frontend_, nh_private_);

    //// sampling algorithms
    UniformSampling::Ptr uniform_sampling;
    NormalSampling::Ptr  normal_sampling;
    Resampling::Ptr      resampling;


    loader.load<UniformSampling,  std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling, map_providers_, tf_provider_backend_, nh_private_);
    loader.load<NormalSampling,   std::map<std::string, MapProvider::Ptr>, TFProvider::Ptr, ros::NodeHandle&>(normal_sampling, map_providers_,  tf_provider_backend_, nh_private_);
    loader.load<Resampling, UniformSampling::Ptr, ros::NodeHandle&>(resampling, uniform_sampling, nh_private_);

    particle_filter_.setup(nh_private_,
                           tf_provider_backend_);
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

