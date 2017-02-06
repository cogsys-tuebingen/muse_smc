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
    for(auto &d : data_providers_) {
        d.second->enable();
    }
    particle_filter_->start();
    ros::spin();
}

bool MuseAMCLNode::requestGlobalInitialization(const muse_amcl::GlobalInitializationRequest &req,
                                               muse_amcl::GlobalInitializationResponse &res)
{
    particle_filter_->requestGlobalInitialization();
    return true;
}

bool MuseAMCLNode::requestPoseInitialization(const muse_amcl::PoseInitializationRequest &req,
                                             muse_amcl::PoseInitializationResponse &res)
{
    auto convert_pose = [&req]() {
        tf::Pose p;
        tf::poseMsgToTF(req.pose.pose, p);
        return math::Pose(p);
    };
    auto convert_covariance = [&req]() {
        std::vector<double> v(36, 0.0);
        for(std::size_t i = 0 ; i < 36 ; ++i) {
            v[i] = req.pose.covariance[i];
        }
        return math::Covariance(v);
    };
    particle_filter_->requestPoseInitialization(convert_pose(), convert_covariance());
    return true;
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
    loader.load<UniformSampling,  MapProviders, TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling_, map_providers_, tf_provider_backend_, nh_private_);
    loader.load<NormalSampling,   MapProviders, TFProvider::Ptr, ros::NodeHandle&>(normal_sampling_, map_providers_,  tf_provider_backend_, nh_private_);
    loader.load<Resampling, UniformSampling::Ptr, ros::NodeHandle&>(resampling_, uniform_sampling_, nh_private_);

    //// set up the necessary functions for the particle filter
    particle_filter_.reset(new ParticleFilter);
    particle_filter_->setup(nh_private_,
                           tf_provider_backend_);
    particle_filter_->setNormalsampling(normal_sampling_);
    particle_filter_->setUniformSampling(uniform_sampling_);
    particle_filter_->setResampling(resampling_);
    particle_filter_->requestGlobalInitialization();

    //// set up the froward data functionality
    predicition_forwarder_.reset(new PredictionForwarder(prediction_model_, particle_filter_));
    update_forwarder_.reset(new UpdateForwarder(update_models_, particle_filter_));
    predicition_forwarder_->bind(data_providers_, nh_private_);
    update_forwarder_->bind(data_providers_, map_providers_, nh_private_);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl");
    MuseAMCLNode node;


    return 0;
}

