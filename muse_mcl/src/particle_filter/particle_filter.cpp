#include <muse_mcl/particle_filter/particle_filter.hpp>
#include <muse_mcl/math/angle.hpp>

using namespace muse_mcl;

ParticleFilter::ParticleFilter()  :
    name_("particle_filter"),
    resampling_cycle_(0),
    working_(false),
    stop_working_(true),
    update_cycle_(0),
    abs_motion_integral_linear_resampling_(0.0),
    abs_motion_integral_angular_resampling_(0.0),
    abs_motion_integral_linear_update_(0.0),
    abs_motion_integral_angular_update_(0.0),
    request_pose_initilization_(false),
    request_global_initialization_(false)
{
}

ParticleFilter::~ParticleFilter()
{
    stop_working_ = true;
    notify_event_.notify_one();
    if(worker_thread_.joinable())
        worker_thread_.join();

    tf_publisher_->end();
}

void ParticleFilter::setup(ros::NodeHandle &nh_private,
                           const TFProvider::Ptr &tf_provider)
{
    const double pub_rate_poses     = nh_private.param<double>("pub_rate_poses", 30.0);
    const double pub_rate_tf        = nh_private.param<double>("pub_rate_tf", 30.0);
    const double resolution_linear  = nh_private.param<double>(privateParameter("resolution_linear"), 0.1);
    const double resolution_angular = math::angle::toRad(nh_private.param<double>(privateParameter("resolution_angular"), 10.0));
    const double array_extent       = nh_private.param<double>(privateParameter("array_extent"), 5.0);

    //// SAMPLE SIZE SETUP
    const std::size_t sample_size   = nh_private.param<int>(privateParameter("sample_size"), 0);
    std::size_t sample_size_maximum = sample_size;
    std::size_t sample_size_minimum = sample_size;

    if(sample_size == 0) {
        sample_size_minimum = nh_private.param<int>(privateParameter("sample_size_minimum"), 0);
        sample_size_maximum = nh_private.param<int>(privateParameter("sample_size_maximum"), 0);
    }

    if(sample_size_minimum == 0 || sample_size_maximum == 0) {
        throw std::runtime_error("[ParticleFilter]: Minimum or maximum sample sizes cannot be less than zero!");
    }

    if(sample_size_maximum == 0) {
        throw std::runtime_error("[ParticleFilter]: The maximum sample size may not be zero!");
    }

    if(sample_size_minimum > sample_size_maximum) {
        throw std::runtime_error("[ParticleFilter]: The minimum sample size may not be greater than the maximum sample size!");
    }

    //// RESAMPLING
    resampling_threshold_linear_  = nh_private.param(privateParameter("resampling_threshold_linear"), 0.075);
    resampling_threshold_angular_ = math::angle::toRad(nh_private.param(privateParameter("resampling_threshold_angular"), 5.0));
    resampling_cycle_             = nh_private.param(privateParameter("resampling_cycle"), 2);

    //// FILTER STATE PUBLICATION
    pub_poses_delay_              = ros::Rate(pub_rate_poses).cycleTime();

    //// FRAMES
    world_frame_                  = nh_private.param<std::string>("world_frame", "/world");
    odom_frame_                   = nh_private.param<std::string>("odom_frame", "/odom");
    base_frame_                   = nh_private.param<std::string>("base_frame", "/base_link");

    integrate_all_measurement_    = nh_private.param<bool>(privateParameter("integrate_all_measurement"), false);

    //// SETUP TF
    tf_provider_               = tf_provider;
    tf_publisher_.reset(new TransformPublisher(pub_rate_tf, odom_frame_, base_frame_, world_frame_));

    //// FILTER STATE
    filter_state_publisher_.reset(new FilterStatePublisher(world_frame_));

    muse_mcl::Indexation indexation ({resolution_linear, resolution_linear, resolution_angular});
    particle_set_.reset(new ParticleSet(world_frame_, sample_size_minimum, sample_size_maximum, indexation, array_extent));
    particle_set_stamp_ = ros::Time::now(); /// the first time the particle set is touched is the first time it is valid

    tf_latest_w_T_b_ = tf::StampedTransform(tf::Transform::getIdentity(),
                                            particle_set_stamp_,
                                            base_frame_,
                                            world_frame_);


    //// LOGGING ////
    const double now = ros::Time::now().toSec();
    FilterStateLoggerDefault::Header header = {"predictions, updates, driven_linear, driven_angular, time_ratio"};
    filter_state_logger_.reset(new FilterStateLoggerDefault(header));
    filter_state_logger_->log(prediction_queue_.size(),
                              update_queue_.size(),
                              abs_motion_integral_linear_resampling_,
                              abs_motion_integral_angular_resampling_,
                              particle_set_stamp_.toSec() / now);
    dotty_.reset(new Dotty);
    ////////////////

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
    {
        std::unique_lock<std::mutex> l(prediction_queue_mutex_);
        prediction_queue_.emplace(prediction);
        notify_prediction_.notify_one();
    }
    saveFilterState();
}

void ParticleFilter::addUpdate(Update::Ptr &update)
{
    std::unique_lock<std::mutex> l(update_queue_mutex_);
    update_queue_.emplace(update);
    notify_event_.notify_one();

    saveFilterState();
}

void ParticleFilter::requestPoseInitialization(const math::Pose &pose,
                                               const math::Covariance &covariance)
{
    {
        std::unique_lock<std::mutex> l(request_pose_mutex_);
        initialization_pose_ = pose;
        initialization_covariance_ = covariance;
        request_pose_initilization_ = true;
        notify_event_.notify_one();
    }
}

void ParticleFilter::requestGlobalInitialization()
{
    request_global_initialization_ = true;
    notify_event_.notify_one();
}

void ParticleFilter::start()
{
    if(!working_) {
        std::unique_lock<std::mutex> l(worker_thread_mutex_);
        stop_working_  = false;
        worker_thread_ = std::thread([this](){loop();});
        worker_thread_.detach();
        working_ = true;

        tf_publisher_->start();
    } else {
        throw std::runtime_error("[ParticleFilter]: Worker thread already running!");
    }
}

void ParticleFilter::end()
{
    if(working_) {
        std::unique_lock<std::mutex> l(worker_thread_mutex_);
        stop_working_ = true;
        notify_event_.notify_one();
        if(worker_thread_.joinable())
            worker_thread_.join();

        tf_publisher_->end();
    } else{
        throw std::runtime_error("[ParticleFilter]: Cannot end worker thread which is not running!");
    }
}

void ParticleFilter::processRequests()
{
    if(request_global_initialization_) {
        const ros::Time now = ros::Time::now();
        request_global_initialization_ = false;
        sampling_uniform_->apply(*particle_set_);

        if(particle_set_stamp_ > now) {
            update_queue_ = UpdateQueue();
            prediction_queue_ = PredictionQueue();
        }

        particle_set_stamp_ = now;

        publishPoses();
    }
    if(request_pose_initilization_) {
        const ros::Time now = ros::Time::now();
        std::unique_lock<std::mutex> l(request_pose_mutex_);
        request_pose_initilization_ = false;
        sampling_normal_pose_->apply(initialization_pose_, initialization_covariance_, *particle_set_);

        if(particle_set_stamp_ > now) {
            update_queue_ = UpdateQueue();
            prediction_queue_ = PredictionQueue();
        }

        particle_set_stamp_ = now;

        tf_latest_w_T_b_ = tf::StampedTransform(initialization_pose_.getPose(), particle_set_stamp_, base_frame_, world_frame_);
        publishPoses();
        publishTF();
    }
}

void ParticleFilter::processPredictions(const ros::Time &until)
{
    auto wait_for_prediction = [this] ()
    {
        std::unique_lock<std::mutex> l(notify_prediction_mutex_);
        notify_prediction_.wait(l);
    };

    double local_abs_motion_integral_linear = 0.0;
    double local_abs_motion_integral_angular = 0.0;

    while(until > particle_set_stamp_) {
        Prediction::Ptr prediction;
        {
            if(prediction_queue_.empty()) {
                wait_for_prediction();
            }

            std::unique_lock<std::mutex> l(prediction_queue_mutex_);
            prediction = prediction_queue_.top();
            prediction_queue_.pop();
        }

        saveFilterState();

        /// remove too old predction messages
        if(prediction->getStamp() < particle_set_stamp_) {
            continue;
        }

        /*
         * There must be more logic behind this.
         * -> we have to check if motion was fully applied
         * +-> if so check if the timestamp is equal to 'until'
         * -> split and retries have to be handled.
         */

        /// mutate time stamp
        PredictionModel::Result movement = prediction->apply(until, particle_set_->getPoses());
        if(movement.success()) {
            local_abs_motion_integral_linear  += movement.linear_distance_abs;
            local_abs_motion_integral_angular += movement.angular_distance_abs;

            particle_set_stamp_                = movement.applied->getTimeFrame().end;

            dotty_->addPrediction(movement.applied->getTimeFrame().end, static_cast<bool>(movement.left_to_apply));


            if(movement.left_to_apply) {
                Prediction::Ptr prediction_left_to_apply
                        (new Prediction(movement.left_to_apply, prediction->getPredictionModel()));
                std::unique_lock<std::mutex> l(prediction_queue_mutex_);
                prediction_queue_.emplace(prediction_left_to_apply);
                break;
            }
        } else {
            std::unique_lock<std::mutex> l(prediction_queue_mutex_);
            prediction_queue_.emplace(prediction);
        }
    }

    abs_motion_integral_linear_resampling_  += local_abs_motion_integral_linear;
    abs_motion_integral_angular_resampling_ += local_abs_motion_integral_angular;
    abs_motion_integral_linear_update_      += local_abs_motion_integral_linear;
    abs_motion_integral_angular_update_     += local_abs_motion_integral_angular;

    saveFilterState();
}

void ParticleFilter::publishPoses()
{
    filter_state_publisher_->addState(particle_set_->getSamples(), particle_set_mean_, particle_set_stamp_);
}

void ParticleFilter::publishTF()
{
    tf_publisher_->setTransform(tf_latest_w_T_b_);
}

void ParticleFilter::loop()
{
    std::unique_lock<std::mutex> lock_notify(notify_mutex_);
    while(particle_set_stamp_.isZero())
        particle_set_stamp_ = ros::Time::now();

    while(!stop_working_) {
        notify_event_.wait(lock_notify);

        if(stop_working_)
            break;

        if(particle_set_->getSampleSize() == 0) {
            sampling_uniform_->apply(*particle_set_);
        }

        while(updatesQueued()) {
            if(stop_working_)
                break;

            processRequests();

            Update::Ptr update = getUpdate();
            auto time = update->getStamp();
            if(time >= particle_set_stamp_) {
                processPredictions(time);
                if(time > particle_set_stamp_) {
                    queueUpdate(update);
                } else if(time == particle_set_stamp_) {
                    if(integrate_all_measurement_ ||
                            abs_motion_integral_linear_update_ > 0.0 ||
                                abs_motion_integral_angular_update_ > 0.0) {
                        applyUpdate(update);
                    }
                    abs_motion_integral_linear_update_ = 0.0;
                    abs_motion_integral_angular_update_ = 0.0;
                } else  {
                    std::cerr << "Motion model seems not to have interpolated to update stamp!" << std::endl;
                }
                ++update_cycle_;
            }

            saveFilterState();
            tryToResample();
            saveFilterState();
            publishPoses();
        }
    }

    working_ = false;
    saveFilterState();
}

void ParticleFilter::tryToResample()
{
    /// 6. check if its time for resampling
    const bool motion_criterion = abs_motion_integral_linear_resampling_ > resampling_threshold_linear_ ||
            abs_motion_integral_angular_resampling_ > resampling_threshold_angular_;
    const bool cycle_criterion =  resampling_cycle_ > 0 && update_cycle_ >= resampling_cycle_ &&
            (abs_motion_integral_linear_resampling_ > 0.0 || abs_motion_integral_angular_resampling_ > 0.0);

    if(motion_criterion || cycle_criterion){
        resampling_->apply(*particle_set_);
        particle_set_->normalizeWeights();

        abs_motion_integral_linear_resampling_  = 0.0;
        abs_motion_integral_angular_resampling_ = 0.0;

        /// 7. cluster the particle set an update the transformation
        /// @todo allow mean method
        particle_set_->cluster();
        const ParticleSet::Clusters      &clusters      = particle_set_->getClusters();
        const ParticleSet::Distributions &distributions = particle_set_->getClusterDistributions();
        const ParticleSet::AngularMeans  &angular_means = particle_set_->getClusterAngularMeans();

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

        if(max_cluster_id != -1) {
            particle_set_->resetWeights();
            particle_set_mean_ = math::Pose(distributions.at(max_cluster_id).getMean(),
                                            angular_means.at(max_cluster_id).getMean());

            tf_latest_w_T_b_ = tf::StampedTransform(particle_set_mean_.getPose(),
                                                    particle_set_stamp_, world_frame_, base_frame_);
            publishTF();
        }

        update_cycle_ = 0;
    }
}
