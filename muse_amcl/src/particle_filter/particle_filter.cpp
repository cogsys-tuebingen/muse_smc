#include <muse_amcl/particle_filter/particle_filter.hpp>

using namespace muse_amcl;

ParticleFilter::ParticleFilter()  :
    name_("particle_filter"),
    pub_poses_last_time_(ros::Time::now()),
    pub_tf_last_time_(ros::Time::now()),
    working_(false),
    stop_working_(true),
    request_global_initialization_(false),
    request_pose_initilization_(false),
    prediction_linear_distance_(0.0),
    prediction_angular_distance_(0.0)
{
}

ParticleFilter::~ParticleFilter()
{
    stop_working_ = true;
    notify_event_.notify_one();
    if(worker_thread_.joinable())
        worker_thread_.join();
}

void ParticleFilter::setup(ros::NodeHandle &nh_private,
                           const TFProvider::Ptr &tf_provider)
{
    const std::string topic_poses   = nh_private.param<std::string>(privateParameter("topic_poses"), "/muse_amcl/poses");
    const double pose_rate          = nh_private.param<double>("pose_rate", 30.0);
    const double pub_tf_rate        = nh_private.param<double>("tf_rate", 30.0);
    const double resolution_linear  = nh_private.param<double>(privateParameter("resolution_linear"), 0.1);
    const double resolution_angular = nh_private.param<double>(privateParameter("resolution_angular"), M_PI / 18.0);
    const double array_extent       = nh_private.param<double>(privateParameter("array_extent"), 5.0);
    const std::size_t sample_size   = nh_private.param<int>(privateParameter("sample_size"), 0);
    std::size_t sample_size_maximum = sample_size;
    std::size_t sample_size_minimum = sample_size;

    if(sample_size == 0) {
        sample_size_maximum = nh_private.param<int>(privateParameter("sample_size_minimum"), 0);
        sample_size_minimum = nh_private.param<int>(privateParameter("sample_size_maximum"), 0);
    }

    if(sample_size_minimum < 0 || sample_size_maximum < 0) {
        throw std::runtime_error("[ParticleFilter]: Minimum or maximum sample sizes cannot be less than zero!");
    }

    if(sample_size_maximum == 0) {
        throw std::runtime_error("[ParticleFilter]: The maximum sample size may not be zero!");
    }

    if(sample_size_minimum > sample_size_maximum) {
        throw std::runtime_error("[ParticleFilter]: The minimum sample size may not be greater than the maximum sample size!");
    }

    tf_provider_ = tf_provider;

    resampling_offset_linear_  = nh_private.param(privateParameter("resampling_offset_linear"), 0.25);
    resampling_offset_angular_ = nh_private.param(privateParameter("resampling_offset_angular"), M_PI / 10.0);
    pub_poses_                 = nh_private.advertise<geometry_msgs::PoseArray>(topic_poses, 1);
    pub_poses_delay_           = ros::Rate(pose_rate).cycleTime();
    pub_tf_delay_              = ros::Rate(pub_tf_rate).cycleTime();

    world_frame_   = nh_private.param<std::string>("world_frame", "/world");
    odom_frame_    = nh_private.param<std::string>("odom_frame", "/odom");
    base_frame_    = nh_private.param<std::string>("base_frame", "/base_link");

    muse_amcl::Indexation indexation ({resolution_linear, resolution_linear, resolution_angular});
    particle_set_.reset(new ParticleSet(world_frame_, sample_size_minimum, sample_size_maximum, indexation, array_extent));
    particle_set_stamp_ = ros::Time::now(); /// the first time the particle set is touched is the first time it is valid
}

void ParticleFilter::setUniformSampling(const UniformSampling::Ptr &sampling_uniform)
{
    sampling_uniform_ = sampling_uniform;
}

UniformSampling::Ptr ParticleFilter::getUniformSampling() const
{
    return sampling_uniform_;
}

void ParticleFilter::setNormalsampling(NormalSampling::Ptr &sampling_normal_pose)
{
    sampling_normal_pose_ = sampling_normal_pose;
}

NormalSampling::Ptr ParticleFilter::getNormalSampling() const
{
    return sampling_normal_pose_;
}

void ParticleFilter::setResampling(Resampling::Ptr &resampling)
{
    resampling_ = resampling;
}

Resampling::Ptr ParticleFilter::getResampling() const
{
    return resampling_;
}

void ParticleFilter::addPrediction(Prediction::Ptr &prediction)
{
    std::unique_lock<std::mutex> l(prediction_queue_mutex_);
    prediction_queue_.emplace(prediction);
    //    notify_event_.notify_one();   <--- main trigger should be the sensor measurements
}

void ParticleFilter::addUpdate(Update::Ptr &update)
{
    std::unique_lock<std::mutex> l(update_queue_mutex_);
    update_queue_.emplace(update);
    notify_event_.notify_one();
}

void ParticleFilter::requestPoseInitialization(const math::Pose &pose,
                                               const math::Covariance &covariance)
{
    std::cout << "[ParticleFilter]: requested pose localization" << std::endl;
    std::unique_lock<std::mutex> l(request_pose_mutex_);
    initialization_pose_ = pose;
    initialization_covariance_ = covariance;
    request_pose_initilization_ = true;
    particle_set_stamp_ = ros::Time::now();
    notify_event_.notify_one();
}

void ParticleFilter::requestGlobalInitialization()
{
    std::cout << "[ParticleFilter]: requested global localization" << std::endl;
    request_global_initialization_ = true;
    particle_set_stamp_ = ros::Time::now();
    notify_event_.notify_one();
}

void ParticleFilter::start()
{
    std::unique_lock<std::mutex> l(worker_thread_mutex_);
    if(!working_) {
        stop_working_  = false;
        worker_thread_ = std::thread([this](){loop();});
        worker_thread_.detach();
        working_ = true;
    } else {
        throw std::runtime_error("[ParticleFilter]: Worker thread already running!");
    }
}

void ParticleFilter::end()
{
    std::unique_lock<std::mutex> l(worker_thread_mutex_);
    if(working_) {
        stop_working_ = true;
        notify_event_.notify_one();
        if(worker_thread_.joinable())
            worker_thread_.join();
    } else{
        throw std::runtime_error("[ParticleFilter]: Cannot end worker thread which is not running!");
    }
}

void ParticleFilter::processRequests()
{
    if(request_global_initialization_) {
        request_global_initialization_ = false;
        sampling_uniform_->apply(*particle_set_);
        particle_set_stamp_ = ros::Time::now();
        publishPoses(true);
    }
    if(request_pose_initilization_) {
        std::unique_lock<std::mutex> l(request_pose_mutex_);
        request_pose_initilization_ = false;
        sampling_normal_pose_->apply(initialization_pose_, initialization_covariance_, *particle_set_);
        particle_set_stamp_ = ros::Time::now();
        publishPoses(true);
    }
}

bool ParticleFilter::processPredictions(const ros::Time &until)
{
    while(until > particle_set_stamp_) {
        Prediction::Ptr prediction;
        {
            std::unique_lock<std::mutex> l(prediction_queue_mutex_);
            if(prediction_queue_.empty())
                return false;

            prediction = prediction_queue_.top();
            prediction_queue_.pop();
        }
        /// remove too old predction messages
        if(prediction->getStamp() < particle_set_stamp_)
            continue;

        /// mutate time stamp

        PredictionModel::Movement movement = prediction->apply(until, particle_set_->getPoses());
        prediction_linear_distance_       += movement.linear_distance;
        prediction_angular_distance_      += movement.angular_distance;
        particle_set_stamp_               += movement.prediction_duration;

        if(!prediction->isDone()) {
            /// if prediction is not fully finished, push it back onto the queue
            std::unique_lock<std::mutex> l(prediction_queue_mutex_);
            prediction_queue_.emplace(prediction);
            break;
        }
    }

    return prediction_linear_distance_ > 0.0 || prediction_angular_distance_ > 0.0;
}

void ParticleFilter::publishPoses(const bool force)
{
    const ros::Time now = ros::Time::now();
    if(pub_poses_last_time_ + pub_poses_delay_ < now || force) {
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = particle_set_->getFrame();
        poses.header.stamp = now;

        auto pose_to_msg = [] (const Particle &particle)
        {
            geometry_msgs::Pose pose;
            tf::poseTFToMsg(particle.pose_.getPose(), pose);
            return pose;
        };

        for(auto &sample : particle_set_->getSamples()) {
            poses.poses.emplace_back(pose_to_msg(sample));
        }
        pub_poses_.publish(poses);
        pub_poses_last_time_ = now;
    };
}

void ParticleFilter::publishTF()
{
    const ros::Time now = ros::Time::now();
    tf::Transform o_T_b;
    if(tf_provider_->lookupTransform(odom_frame_, base_frame_, now, o_T_b, ros::Duration(0.1))) {
        tf::StampedTransform o_T_w((o_T_b * tf_last_b_T_w_).inverse(), now, world_frame_, odom_frame_);
        tf_broadcaster_.sendTransform(o_T_w);
    }
}

void ParticleFilter::loop()
{
    std::unique_lock<std::mutex> lock_updates(update_queue_mutex_);
    while(!stop_working_) {
        /// 0. wait for new tasks
        notify_event_.wait(lock_updates);
        /// 1. check if we want to terminate the filter
        if(stop_working_) {
            break;
        }
        /// 1. process requests if possible
        processRequests();
        if(particle_set_->getSampleSize() == 0) {
            sampling_uniform_->apply(*particle_set_);
        }
        /// 2. check for new update in the queue
        if(!update_queue_.empty()) {
            Update::Ptr update = update_queue_.top();
            update_queue_.pop();
            /// 4. drop it if it is too old
            if(update->getStamp() >= particle_set_stamp_) {
                /// 5. drop it if there was no movement at all
                if(processPredictions(update->getStamp())) {
                    update->apply(particle_set_->getWeights());
                    particle_set_->normalizeWeights();
                }
            }
        }

        /// 6. check if its time for resampling
        if(prediction_linear_distance_ > resampling_offset_linear_ ||
                prediction_angular_distance_ > resampling_offset_angular_){
            particle_set_->normalizeWeights();
            resampling_->apply(*particle_set_);

            for(auto &weight : particle_set_->getWeights()) {
                weight = 1.0;
            }

            prediction_linear_distance_  = 0.0;
            prediction_angular_distance_ = 0.0;

            /// 7. cluster the particle set an update the transformation
            /// @todo allow mean method
            particle_set_->cluster();
            const ParticleSet::Clusters &clusters = particle_set_->getClusters();
            const ParticleSet::Distributions &distributions = particle_set_->getClusterDistributions();
            /// todo get the transformation odom -> world
            double max_weight = std::numeric_limits<double>::lowest();
            int    max_cluster_id = -1;
            for(const auto &cluster : clusters) {
                const int cluster_id = cluster.first;
                const auto &distribution = distributions.at(cluster_id);
                const double weight = distribution.getWeight() ;
                if(weight > max_weight) {
                    max_cluster_id = cluster_id;
                    max_weight = weight;
                }
            }

            if(max_cluster_id == -1) {
                std::cerr << "[ParticleFilter]: Clustering the particles seems to has failed!" << std::endl;
            } else {
                Eigen::Vector3d mean = distributions.at(max_cluster_id).getMean();
                math::Pose w_T_b(mean);
                tf_last_b_T_w_ = w_T_b.inverse().getPose(), particle_set_stamp_;
            }
       }
        publishPoses(true);
//        publishTF();
    }
    working_ = false;

}

