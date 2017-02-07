#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/particle_filter/resampling.hpp>
#include <muse_amcl/particle_filter/sampling_normal.hpp>
#include <muse_amcl/particle_filter/sampling_uniform.hpp>
#include <muse_amcl/particle_filter/prediction.hpp>
#include <muse_amcl/particle_filter/update.hpp>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>

namespace muse_amcl {
class ParticleFilter {
public:
    using Ptr = std::shared_ptr<ParticleFilter>;
    using UpdateQueue     =
    std::priority_queue<Update::Ptr, std::vector<Update::Ptr>, Update::Less>;
    using PredictionQueue =
    std::priority_queue<Prediction::Ptr, std::vector<Prediction::Ptr>, Prediction::Less>;

    ParticleFilter() :
        name_("particle_filter"),
        pub_poses_last_time_(ros::Time::now()),
        pub_tf_last_time_(ros::Time::now()),
        working_(false),
        stop_working_(true)
    {
    }

    virtual ~ParticleFilter()
    {
        stop_working_ = true;
        notify_event_.notify_one();
    }

    void setup(ros::NodeHandle &nh_private,
               const TFProvider::Ptr &tf_provider)
    {
        tf_provider_ = tf_provider;

        resampling_offset_linear_  =
                nh_private.param(privateParameter("resampling_offset_linear"), 0.25);
        resampling_offset_angular_ =
                nh_private.param(privateParameter("resampling_offset_angular"), M_PI / 10.0);

        const std::string topic_poses = nh_private.param<std::string>(privateParameter("topic_poses"), "/muse_amcl/poses");
        const std::string world_frame = nh_private.param<std::string>("world_frame", "/world");
        const double pose_rate = nh_private.param<double>("pose_rate", 30.0);
        const double pub_tf_rate = nh_private.param<double>("tf_rate", 30.0);
        const double resolution_linear  = nh_private.param<double>(privateParameter("resolution_linear"), 0.1);
        const double resolution_angular = nh_private.param<double>(privateParameter("resolution_angular"), M_PI / 18.0);
        const double array_extent       = nh_private.param<double>(privateParameter("array_extent"), 5.0);
        const std::size_t sample_size = nh_private.param<int>(privateParameter("sample_size"), 0);
        std::size_t sample_size_maximum = sample_size;
        std::size_t sample_size_minimum = sample_size;

        if(sample_size == 0) {
           sample_size_maximum = nh_private.param<int>(privateParameter("sample_size_minimum"), 0);
           sample_size_minimum = nh_private.param<int>(privateParameter("sample_size_maximum"), 0);
        }

        pub_poses_  = nh_private.advertise<geometry_msgs::PoseArray>(topic_poses, 1);
        pub_poses_delay_ = ros::Rate(pose_rate).cycleTime();
        pub_tf_delay_ = ros::Rate(pub_tf_rate).cycleTime();

        muse_amcl::Indexation indexation ({resolution_linear, resolution_linear, resolution_angular});
        particle_set_.reset(new ParticleSet(world_frame, sample_size_minimum, sample_size_maximum, indexation, array_extent));
    }

    /// uniform pose sampling
    void setUniformSampling(UniformSampling::Ptr &sampling_uniform)
    {
        sampling_uniform_ = sampling_uniform;

    }
    UniformSampling::Ptr getUniformSampling() const
    {
        return sampling_uniform_;
    }

    /// normal pose sampling
    void setNormalsampling(NormalSampling::Ptr &sampling_normal_pose)
    {
        sampling_normal_pose_ = sampling_normal_pose;
    }
    NormalSampling::Ptr getNormalSampling() const
    {
        return sampling_normal_pose_;
    }

    /// resampling
    void setResampling(Resampling::Ptr &resampling)
    {
        resampling_ = resampling;
    }
    Resampling::Ptr getResampling() const
    {
        return resampling_;
    }

    /// insert new predictions
    void addPrediction(Prediction::Ptr &prediction)
    {
        std::unique_lock<std::mutex> l(prediction_queue_mutex_);
        prediction_queue_.emplace(prediction);
        notify_event_.notify_one();
    }

    void addUpdate(Update::Ptr &update)
    {
        std::unique_lock<std::mutex> l(update_queue_mutex_);
        update_queue_.emplace(update);
        notify_event_.notify_one();
    }

    void requestPoseInitialization(const math::Pose &pose,
                                   const math::Covariance &covariance)
    {
        std::unique_lock<std::mutex> l(request_pose_mutex_);
        initialization_pose_ = pose;
        initialization_covariance_ = covariance;
        request_pose_initilization_ = true;
        notify_event_.notify_one();
    }

    void requestGlobalInitialization()
    {
        request_global_initialization_ = true;
        notify_event_.notify_one();
    }

    void start()
    {
        if(!working_) {
            stop_working_ = false;
            worker_thread_ = std::thread([this](){loop();});
            requestGlobalInitialization();
        }
    }

//    void end()
//    {
//        if(working_) {
//            notify_event_.notify_one();
//        }
//    }

protected:
    std::string             name_;

    ros::Publisher          pub_poses_;
    ros::Time               pub_poses_last_time_;
    ros::Time               pub_tf_last_time_;

    TFProvider::Ptr         tf_provider_;
    ros::Time               particle_set_stamp_;
    ParticleSet::Ptr        particle_set_;

    UniformSampling::Ptr    sampling_uniform_;
    NormalSampling::Ptr     sampling_normal_pose_;
    Resampling::Ptr         resampling_;

    std::thread             worker_thread_;
    std::atomic_bool        working_;
    std::atomic_bool        stop_working_;
    std::condition_variable notify_event_;

    //// ------------------ parameters ----------------------///
    double                  resampling_offset_linear_;
    double                  resampling_offset_angular_;
    ros::Duration           pub_poses_delay_;
    ros::Duration           pub_tf_delay_;


    //// ------------------ working members ----------------///
    mutable std::mutex      update_queue_mutex_;
    mutable std::mutex      prediction_queue_mutex_;
    UpdateQueue             update_queue_;                  /// this is for the weighting functions and therefore important
    PredictionQueue         prediction_queue_;              /// the predcition queue may not reach ovbersize.

    double                  prediction_linear_distance_;    /// integrated movement from odometry
    double                  prediction_angular_distance_;   /// integrated movement from odometry

    //// ------------------ requests -----------------------///
    std::mutex              request_pose_mutex_;
    math::Pose              initialization_pose_;
    math::Covariance        initialization_covariance_;
    std::atomic_bool        request_pose_initilization_;
    std::atomic_bool        request_global_initialization_;

    inline void processRequests()
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

    inline bool predict(const ros::Time &until)
    {
        /// as long we haven't predicted far enough in time, we need to do so
        /// if we get an left over or reached the time, we can drop out
        while(until > particle_set_stamp_) {
            Prediction::Ptr prediction;
            {
                std::unique_lock<std::mutex> l(prediction_queue_mutex_);
                if(prediction_queue_.empty())
                    return false;

                prediction = prediction_queue_.top();
                prediction_queue_.pop();
            }

            PredictionModel::Movement movement = prediction->apply(until, particle_set_->getPoses());
            prediction_linear_distance_  += movement.linear_distance;
            prediction_angular_distance_ += movement.angular_distance;

            if(!prediction->isDone()) {
                /// if prediction is not fully finished, push it back onto the queue
                std::unique_lock<std::mutex> l(prediction_queue_mutex_);
                prediction_queue_.emplace(prediction);
                break;
            }
        }
        return true;
    }

    inline void publishPoses(const bool force = false)
    {
        const ros::Time now = ros::Time::now();
        if(pub_poses_last_time_ + pub_poses_delay_ > now || force) {
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
            ros::spinOnce();
            pub_poses_last_time_ = now;
        };
    }

    inline void publishTF()
    {
        const ros::Time now = ros::Time::now();
        if(pub_tf_last_time_ + pub_tf_delay_ > now) {

            pub_tf_last_time_ = now;
        };
    }

    inline void loop()
    {
        std::unique_lock<std::mutex> lock_updates(update_queue_mutex_);
        working_ = true;

        while(!stop_working_) {
            /// 0. wait for new tasks
            notify_event_.wait(lock_updates);
            /// 1. process requests which occured
            processRequests();
            /// 2. get a new sensor update from the queue
            //            if(!update_queue_.empty()) {
            //                Update::Ptr update = update_queue_.top();
            //                update_queue_.pop();
            //                lock_updates.unlock();
            //                /// 4. propagate until we reach the time stamp of update
            //                if(predict(update->getStamp())) {
            //                    update->apply(particle_set_->getWeights());
            //                    update.reset();
            //                }
            //                /// 5. go on with the queue
            //                lock_updates.lock();
            //                if(update)
            //                    update_queue_.emplace(update);
            //            }
            //            /// 6. check if its time for resampling
            //            if(prediction_linear_distance_ > resampling_minimum_linear_distance_ ||
            //                    prediction_angular_distance_ > resampling_minimum_angular_distance_){
            //                prediction_linear_distance_  = 0.0;
            //                prediction_angular_distance_ = 0.0;

            //                resampling_->apply(*particle_set_);

            //                for(auto &weight : particle_set_->getWeights()) {
            //                    weight = 1.0;
            //                }

            //                /// 7. cluster the particle set an update the transformation
            //                particle_set_->cluster();
            //                ParticleSet::Clusters clusters = particle_set_->getClusters();


            //                /// todo get the transformation odom -> world

            //            }
            publishPoses();
            publishTF();
        }
        working_ = false;
    }

    std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
