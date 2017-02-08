#include "muse_amcl_node.h"

using namespace muse_amcl;

MuseAMCLNode::MuseAMCLNode() :
    nh_private_("~"),
    tf_provider_frontend_(new TFProvider),
    tf_provider_backend_(new TFProvider)
{
}

MuseAMCLNode::~MuseAMCLNode()
{
    for(auto &d : data_providers_) {
        d.second->disable();
    }
}

void MuseAMCLNode::start()
{
    for(auto &d : data_providers_) {
        d.second->enable();
    }
    particle_filter_->start();
    std::cout << "[MuseAMCLNode]: Up and running!" << std::endl;
    ros::spin();
}

bool MuseAMCLNode::requestGlobalInitialization(muse_amcl::GlobalInitialization::Request &req,
                                               muse_amcl::GlobalInitialization::Response &res)
{
    particle_filter_->requestGlobalInitialization();
    res.success = true;
    return true;
}

bool MuseAMCLNode::requestPoseInitialization(muse_amcl::PoseInitialization::Request &req,
                                             muse_amcl::PoseInitialization::Response &res)
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
    res.success = true;
    return true;
}

void MuseAMCLNode::poseInitialization(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    auto convert_pose = [&msg]() {
        tf::Pose p;
        tf::poseMsgToTF(msg->pose.pose, p);
        return math::Pose(p);
    };
    auto convert_covariance = [&msg]() {
        std::vector<double> v(36, 0.0);
        for(std::size_t i = 0 ; i < 36 ; ++i) {
            v[i] = msg->pose.covariance[i];
        }
        return math::Covariance(v);
    };
    particle_filter_->requestPoseInitialization(convert_pose(), convert_covariance());
}

bool MuseAMCLNode::setup()
{
    /// load all plugins
    PluginLoader loader(nh_private_);

    loader.load<UpdateModel, TFProvider::Ptr, ros::NodeHandle&>(update_models_, tf_provider_backend_, nh_private_);
    if(update_models_.empty()) {
        std::cerr << "[MuseAMCLNode]: No update models were found!" << std::endl;
        return false;
    }
    loader.load<PredictionModel, TFProvider::Ptr, ros::NodeHandle&>(prediction_model_, tf_provider_backend_, nh_private_);
    if(update_models_.empty()) {
        std::cerr << "[MuseAMCLNode]: No prediction model was found!" << std::endl;
        return false;
    }
    loader.load<MapProvider, ros::NodeHandle&>(map_providers_, nh_private_);
    if(update_models_.empty()) {
        std::cerr << "[MuseAMCLNode]: No map providers were found!" << std::endl;
        return false;
    }
    loader.load<DataProvider,TFProvider::Ptr, ros::NodeHandle&>(data_providers_, tf_provider_frontend_, nh_private_);
    if(update_models_.empty()) {
        std::cerr << "[MuseAMCLNode]: No data providers were found!" << std::endl;
        return false;
    }

    //// sampling algorithms
    loader.load<UniformSampling,  MapProviders, TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling_, map_providers_, tf_provider_backend_, nh_private_);
    if(!uniform_sampling_) {
        std::cerr << "[MuseAMCLNode]: No uniform sampling algorithm was found!" << std::endl;
        return false;
    }
    loader.load<NormalSampling, MapProviders, TFProvider::Ptr, ros::NodeHandle&>(normal_sampling_, map_providers_,  tf_provider_backend_, nh_private_);
    if(!normal_sampling_) {
        std::cerr << "[MuseAMCLNode]: No normal sampling algorithm was found!" << std::endl;
        return false;
    }
    loader.load<Resampling, UniformSampling::Ptr, ros::NodeHandle&>(resampling_, uniform_sampling_, nh_private_);
    if(!resampling_) {
        std::cerr << "[MuseAMCLNode]: No resampling algorithm was found!" << std::endl;
        return false;
    }

    //// set up the necessary functions for the particle filter
    particle_filter_.reset(new ParticleFilter);
    particle_filter_->setup(nh_private_, tf_provider_backend_);
    particle_filter_->setNormalsampling(normal_sampling_);
    particle_filter_->setUniformSampling(uniform_sampling_);
    particle_filter_->setResampling(resampling_);

    //// set up the froward data functionality
    predicition_forwarder_.reset(new PredictionForwarder(prediction_model_, particle_filter_));
    update_forwarder_.reset(new UpdateForwarder(update_models_, particle_filter_));
    predicition_forwarder_->bind(data_providers_, nh_private_);
    update_forwarder_->bind(data_providers_, map_providers_, nh_private_);

    initialization_service_pose_    = nh_private_.advertiseService("/muse_amcl/pose_initialization", &MuseAMCLNode::requestPoseInitialization, this);
    initialization_service_global_  = nh_private_.advertiseService("/muse_amcl/global_initialization", &MuseAMCLNode::requestGlobalInitialization, this);
    initialization_subscriber_pose_ = nh_private_.subscribe("/initialpose", 1, &MuseAMCLNode::poseInitialization, this);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl");
    MuseAMCLNode node;
    if(node.setup())
        node.start();
    return 0;
    }

