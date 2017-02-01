#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <muse_amcl/data_sources/tf_provider.hpp>
#include <muse_amcl/particle_filter/particle_set.hpp>
#include <muse_amcl/particle_filter/resampling.hpp>
#include <muse_amcl/particle_filter/sampling_normal.hpp>
#include <muse_amcl/particle_filter/sampling_uniform.hpp>
#include <muse_amcl/particle_filter/prediction.hpp>
#include <muse_amcl/particle_filter/update.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>


namespace muse_amcl {
class ParticleFilter {
public:
    using Ptr = std::shared_ptr<ParticleFilter>;
    using UpdateQueue = std::priority_queue<Update::Ptr, std::vector<Update::Ptr>, Update::Less>;
    using PredictionQueue = std::priority_queue<Prediction::Ptr, std::vector<Prediction::Ptr>, Prediction::Less>;

    ParticleFilter() :
        name_("particle_filter"),
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
    }

    /// uniform pose sampling
    void setUniformSampling(UniformSampling::Ptr &uniform_sampling)
    {
        uniform_sampling = uniform_sampling;

    }
    UniformSampling::Ptr getUniformSampling() const
    {
        return uniform_sampling_;
    }

    /// normal pose sampling
    void setNormalsampling(NormalSampling::Ptr &normal_sampling)
    {
        normal_sampling_ = normal_sampling;
    }
    NormalSampling::Ptr getNormalSampling() const
    {
        return normal_sampling_;
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

    void requestPoseInitialization(const math::Pose &pose)
    {
        std::unique_lock<std::mutex> l(request_pose_mutex_);
        requset_pose_ = pose;
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
            /// spawn mr. backend thread here
        }
    }

    void end()
    {
        if(working_) {
            notify_event_.notify_one();
        }
    }

protected:
    std::string             name_;
    TFProvider::Ptr         tf_provider_;
    ros::Time               particle_set_stamp_;
    ParticleSet::Ptr        particle_set_;

    UniformSampling::Ptr    uniform_sampling_;
    NormalSampling::Ptr     normal_sampling_;
    Resampling::Ptr         resampling_;

    std::thread             worker_thread_;
    std::atomic_bool        working_;
    std::atomic_bool        stop_working_;
    std::condition_variable notify_event_;

    //// ------------------ paramters ----------------------///
    double                  resampling_minimum_linear_distance_;
    double                  resampling_minimum_angular_distance_;


    //// ------------------ working members ----------------///
    mutable std::mutex      update_queue_mutex_;
    mutable std::mutex      prediction_queue_mutex_;
    UpdateQueue             update_queue_;                  /// this is for the weighting functions and therefore important
    PredictionQueue         prediction_queue_;              /// the predcition queue may not reach ovbersize.

    double                  prediction_linear_distance_;    /// integrated movement from odometry
    double                  prediction_angular_distance_;    /// integrated movement from odometry

    std::mutex              request_pose_mutex_;
    math::Pose              requset_pose_;
    std::atomic_bool        request_pose_initilization_;
    std::atomic_bool        request_global_initialization_;

    inline void processRequests()
    {
        if(request_global_initialization_) {
            request_global_initialization_ = false;
            particle_set_stamp_ = ros::Time::now();
        }
        if(request_pose_initilization_) {
            std::unique_lock<std::mutex> l(request_pose_mutex_);
            request_pose_initilization_ = false;
            particle_set_stamp_ = ros::Time::now();
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
            prediction_linear_distance_ += movement.linear_distance;
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

    inline void publishTF()
    {

    }


    inline void filter()
    {
        std::unique_lock<std::mutex> lock_updates(update_queue_mutex_);
        working_ = true;

        while(!stop_working_) {
            /// 0. wait for new tasks
            notify_event_.wait(lock_updates);
            /// 1. process requests which occured
            processRequests();
            /// 2. get a new sensor update from the queue
            if(!update_queue_.empty()) {
                Update::Ptr update = update_queue_.top();
                update_queue_.pop();
                lock_updates.unlock();
                /// 4. propagate until we reach the time stamp of update
                if(predict(update->getStamp())) {
                    update->apply(particle_set_->getWeights());
                    update.reset();
                }
                /// 5. go on with the queue
                lock_updates.lock();
                if(update)
                    update_queue_.emplace(update);
            }
            /// 6. check if its time for resampling
            if(prediction_linear_distance_ > resampling_minimum_linear_distance_ ||
                    prediction_angular_distance_ > resampling_minimum_angular_distance_){
                prediction_linear_distance_  = 0.0;
                prediction_angular_distance_ = 0.0;

                resampling_->apply(*particle_set_);

                for(auto &weight : particle_set_->getWeights()) {
                    weight = 1.0;
                }

                /// 7. cluster the particle set an update the transformation
                particle_set_->cluster();
                ParticleSet::Clusters clusters = particle_set_->getClusters();
            }
            publishTF();
        }
        working_ = false;
    }

    std::string parameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
