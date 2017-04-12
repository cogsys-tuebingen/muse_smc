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
    Logger& l = Logger::getLogger();
    l.info("Enabled all data providers.", "MuseAMCLNode");

    particle_filter_->start();
    l.info("Started particle filter.", "MuseAMCLNode");
    l.markNewLogSection();

    ros::spin();
}

bool MuseAMCLNode::requestGlobalInitialization(muse_amcl::GlobalInitialization::Request &req,
                                               muse_amcl::GlobalInitialization::Response &res)
{
    Logger& l = Logger::getLogger();
    l.info("Received a global initialization request.", "MuseAMCLNode");

    particle_filter_->requestGlobalInitialization();
    l.info("Dispatched request to particle filter.", "MuseAMCLNode");

    res.success = true;
    return true;
}

bool MuseAMCLNode::requestPoseInitialization(muse_amcl::PoseInitialization::Request &req,
                                             muse_amcl::PoseInitialization::Response &res)
{
    Logger& l = Logger::getLogger();
    l.info("Received a pose initialization request.", "MuseAMCLNode");

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
    l.info("Dispatched request to particle filter.", "MuseAMCLNode");

    res.success = true;
    return true;
}

void MuseAMCLNode::poseInitialization(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
    Logger& l = Logger::getLogger();
    l.info("Received a global initial pose message.", "MuseAMCLNode");

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
    l.info("Dispatched request to particle filter.", "MuseAMCLNode");

}

bool MuseAMCLNode::setup()
{
    Logger& l = Logger::getLogger();
    l.info("Setup started.", "MuseAMCLNode");

    /// load all plugins
    PluginLoader loader(nh_private_);

    loader.load<UpdateModel, TFProvider::Ptr, ros::NodeHandle&>(update_models_, tf_provider_backend_, nh_private_);
    if(update_models_.empty()) {
        l.error("No update models were found!", "MuseAMCLNode");
        return false;
    }
    loader.load<PredictionModel, TFProvider::Ptr, ros::NodeHandle&>(prediction_model_, tf_provider_backend_, nh_private_);
    if(update_models_.empty()) {
        l.error("No prediction model was found!", "MuseAMCLNode");
        return false;
    }
    loader.load<MapProvider, ros::NodeHandle&>(map_providers_, nh_private_);
    if(update_models_.empty()) {
        l.error("No map providers were found!", "MuseAMCLNode");
        return false;
    }
    loader.load<DataProvider,TFProvider::Ptr, ros::NodeHandle&>(data_providers_, tf_provider_frontend_, nh_private_);
    if(update_models_.empty()) {
        l.error("No data providers were found!", "MuseAMCLNode");
        return false;
    }

    //// sampling algorithms
    loader.load<UniformSampling,  MapProviders, TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling_, map_providers_, tf_provider_backend_, nh_private_);
    if(!uniform_sampling_) {
        l.error("No uniform sampling algorithm was found!", "MuseAMCLNode");
        return false;
    }
    loader.load<NormalSampling, MapProviders, TFProvider::Ptr, ros::NodeHandle&>(normal_sampling_, map_providers_,  tf_provider_backend_, nh_private_);
    if(!normal_sampling_) {
        l.error("No normal sampling algorithm was found!", "MuseAMCLNode");
        return false;
    }
    loader.load<Resampling, UniformSampling::Ptr, ros::NodeHandle&>(resampling_, uniform_sampling_, nh_private_);
    if(!resampling_) {
        l.error("No resampling algorithm was found!", "MuseAMCLNode");
        return false;
    }

    l.info("All plugins loaded.", "MuseAMCLNode");

    //// set up the necessary functions for the particle filter
    particle_filter_.reset(new ParticleFilter);
    particle_filter_->setup(nh_private_, tf_provider_backend_);
    particle_filter_->setNormalsampling(normal_sampling_);
    particle_filter_->setUniformSampling(uniform_sampling_);
    particle_filter_->setResampling(resampling_);

    l.info("Equipped particle filter with all providers and samplers.", "MuseAMCLNode");
    //// set up the froward data functionality
    predicition_forwarder_.reset(new PredictionForwarder(prediction_model_, particle_filter_));
    predicition_forwarder_->bind(data_providers_, nh_private_);
    l.info("Equipped prediction forwarder with model.", "MuseAMCLNode");

    update_forwarder_.reset(new UpdateForwarder(update_models_, particle_filter_));
    update_forwarder_->bind(data_providers_, map_providers_, nh_private_);
    l.info("Equipped update forwarder with model.", "MuseAMCLNode");

    initialization_service_pose_    = nh_private_.advertiseService("/muse_amcl/pose_initialization", &MuseAMCLNode::requestPoseInitialization, this);
    initialization_service_global_  = nh_private_.advertiseService("/muse_amcl/global_initialization", &MuseAMCLNode::requestGlobalInitialization, this);
    initialization_subscriber_pose_ = nh_private_.subscribe("/initialpose", 1, &MuseAMCLNode::poseInitialization, this);
    l.info("All subscribers and services set up.", "MuseAMCLNode");
    l.info("Setup has been finished.", "MuseAMCLNode");
    l.markNewLogSection();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_amcl");
    Logger &l = Logger::getLogger(false, Logger::ALL, false);

    MuseAMCLNode node;
    if(node.setup()) {
        node.start();
    } else {
        l.error("Setup failed.", "MuseAMCLNode");
    }
    return 0;
}

