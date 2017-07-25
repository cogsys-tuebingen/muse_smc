#include "muse_mcl_node.h"

using namespace muse_mcl;

MuseMCLNode::MuseMCLNode() :
    nh_private_("~"),
    tf_provider_frontend_(new TFProvider),
    tf_provider_backend_(new TFProvider)
{
}

MuseMCLNode::~MuseMCLNode()
{
    for(auto &d : data_providers_) {
        d.second->disable();
    }
}

void MuseMCLNode::start()
{
    for(auto &d : data_providers_) {
        d.second->enable();
    }

    particle_filter_->start();

    /// check if there is an initial pose set
    checkPoseInitialization();

    double node_rate = nh_private_.param<double>("node_rate", 60.0);
    if(node_rate == 0.0) {
        /// unlimited speed
        ros::spin();
    } else {
        /// limited speed
        ros::WallRate r(node_rate);
        while(ros::ok()) {
            ros::spinOnce();
            r.sleep();
        }
    }
}

bool MuseMCLNode::requestGlobalInitialization(muse_mcl::GlobalInitialization::Request &req,
                                              muse_mcl::GlobalInitialization::Response &res)
{
    particle_filter_->requestGlobalInitialization();
    res.success = true;
    return true;
}

bool MuseMCLNode::requestPoseInitialization(muse_mcl::PoseInitialization::Request &req,
                                            muse_mcl::PoseInitialization::Response &res)
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

void MuseMCLNode::poseInitialization(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
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

bool MuseMCLNode::setup()
{
    /// load all plugins
    PluginLoader loader(nh_private_);

    {   /// Update Models
        loader.load<ModelUpdate, TFProvider::Ptr, ros::NodeHandle&>(update_models_, tf_provider_backend_, nh_private_);
        if(update_models_.empty()) {
            ROS_ERROR_STREAM("No update model functions were found!");
            return false;
        }
        std::string update_model_list = "[";
        for(auto &e : update_models_) {
            update_model_list += e.first + ",";
        }
        update_model_list.at(update_model_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded update function models.");
        ROS_INFO_STREAM(update_model_list);
    }

    {   /// Prediction Model
        loader.load<ModelPrediction, TFProvider::Ptr, ros::NodeHandle&>(prediction_model_, tf_provider_backend_, nh_private_);
        if(!prediction_model_) {
            ROS_ERROR_STREAM("No prediction model functions was found!");
            return false;
        }
        ROS_INFO_STREAM("Loaded prediction function model.");
        ROS_INFO_STREAM("[" << prediction_model_->getName() << "]");
    }
    {   /// Map Providers
        loader.load<ProviderMap, ros::NodeHandle&>(map_providers_, nh_private_);
        if(map_providers_.empty()) {
            ROS_ERROR_STREAM("No map provider was found!");
            return false;
        }
        std::string map_provider_list = "[";
        for(auto &e : map_providers_) {
            map_provider_list += e.first + ",";
        }
        map_provider_list.at(map_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded map providers.");
        ROS_INFO_STREAM(map_provider_list);
    }
    {   /// Data Providers
        loader.load<ProviderData,TFProvider::Ptr, ros::NodeHandle&>(data_providers_, tf_provider_frontend_, nh_private_);
        if(data_providers_.empty()) {
            ROS_ERROR_STREAM("No data provider was found!");
            return false;
        }
        std::string data_provider_list = "[";
        for(auto &e : data_providers_) {
            data_provider_list += e.first + ",";
        }
        data_provider_list.at(data_provider_list.size() - 1) = ']';
        ROS_INFO_STREAM("Loaded data providers.");
        ROS_INFO_STREAM(data_provider_list);
    }
    { /// sampling algorithms
        loader.load<UniformSampling,  MapProviders, TFProvider::Ptr, ros::NodeHandle&>(uniform_sampling_, map_providers_, tf_provider_backend_, nh_private_);
        if(!uniform_sampling_) {
            ROS_ERROR_STREAM("No uniform sampling function was found!");
            return false;
        }
        ROS_INFO_STREAM("Loaded uniform sampler.");
        ROS_INFO_STREAM("[" << uniform_sampling_->getName() << "]");
        loader.load<NormalSampling, MapProviders, TFProvider::Ptr, ros::NodeHandle&>(normal_sampling_, map_providers_,  tf_provider_backend_, nh_private_);
        if(!normal_sampling_) {
            ROS_ERROR_STREAM("No gaussian sampling function was found!");
            return false;
        }
        ROS_INFO_STREAM("Loaded gaussian sampler.");
        ROS_INFO_STREAM("[" << normal_sampling_->getName() << "]");
        loader.load<Resampling, UniformSampling::Ptr, ros::NodeHandle&>(resampling_, uniform_sampling_, nh_private_);
        if(!resampling_) {
            ROS_ERROR_STREAM("No resampling function was found!");
            return false;
        }
        ROS_INFO_STREAM("Loaded resampling alpgorithm.");
        ROS_INFO_STREAM("[" << resampling_->getName() << "]");
    }

    //// set up the necessary functions for the particle filter
    particle_filter_.reset(new ParticleFilter);
    particle_filter_->setup(nh_private_, tf_provider_backend_);
    particle_filter_->setNormalsampling(normal_sampling_);
    particle_filter_->setUniformSampling(uniform_sampling_);
    particle_filter_->setResampling(resampling_);

    //// set up the froward data functionality
    predicition_forwarder_.reset(new PredictionForwarder(prediction_model_, particle_filter_));
    predicition_forwarder_->bind(data_providers_, nh_private_);

    update_forwarder_.reset(new UpdateForwarder(update_models_, particle_filter_));
    update_forwarder_->bind(data_providers_, map_providers_, nh_private_);

    initialization_service_pose_    = nh_private_.advertiseService("/muse_mcl/pose_initialization", &MuseMCLNode::requestPoseInitialization, this);
    initialization_service_global_  = nh_private_.advertiseService("/muse_mcl/global_initialization", &MuseMCLNode::requestGlobalInitialization, this);
    initialization_subscriber_pose_ = nh_private_.subscribe("/initialpose", 1, &MuseMCLNode::poseInitialization, this);

    return true;
}

void MuseMCLNode::checkPoseInitialization()
{
    if(nh_private_.hasParam("initialization/pose") &&
            nh_private_.hasParam("initialization/covariance")) {


        std::vector<double> p_v;
        std::vector<double> c_v;
        nh_private_.getParam("initialization/pose", p_v);
        nh_private_.getParam("initialization/covariance",c_v);

        math::Pose pose;
        bool valid_pose = true;
        switch(p_v.size()) {
        case 6:
            pose = math::Pose(p_v[0], p_v[1], p_v[2],
                    p_v[3], p_v[4], p_v[5]);
            break;
        case 7:
            pose = math::Pose(p_v[0], p_v[1], p_v[2],
                    p_v[3], p_v[4], p_v[5], p_v[6]);
            break;
        default:
            valid_pose = false;
            std::cerr << "[MuseAMCLNode]: Got intial_pose, but invalid count of values." << std::endl;
        }

        if(valid_pose) {
            if(c_v.size() == 36) {
                math::Covariance covariance = math::Covariance(c_v);
                particle_filter_->requestPoseInitialization(pose, covariance);
            } else {
                std::cerr << "[MuseAMCLNode]: Got intial_pose, but invalid count of values." << std::endl;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl");

    MuseMCLNode node;
    if(node.setup()) {
        node.start();
    } else {
        ROS_ERROR_STREAM("[muse_mcl]: Could not set up the node!");
    }
    return 0;
}

