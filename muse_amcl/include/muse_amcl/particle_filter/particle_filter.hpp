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
        stop_working_(true),
        stop_waiting_(false)
    {
    }

    virtual ~ParticleFilter()
    {
        stop_working_ = true;
        stop_waiting_ = true;
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
        std::unique_lock<std::mutex> l(mutex_prediction_queue_);
        prediction_queue_.emplace(prediction);
        notify_event_.notify_one();
    }

    void addUpdate(Update::Ptr &update)
    {
        std::unique_lock<std::mutex> l(mutex_update_queue_);
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
            stop_waiting_ = true;
            notify_event_.notify_one();
        }
    }

protected:
    std::string             name_;
    TFProvider::Ptr         tf_provider_;
    ParticleSet::Ptr        particle_set_;

    UniformSampling::Ptr    uniform_sampling_;
    NormalSampling::Ptr     normal_sampling_;
    Resampling::Ptr         resampling_;


    std::thread             worker_thread_;
    std::atomic_bool        working_;
    std::atomic_bool        stop_working_;
    std::condition_variable notify_event_;

    mutable std::mutex      mutex_update_queue_;
    mutable std::mutex      mutex_prediction_queue_;
    UpdateQueue             update_queue_;      /// this is for the weighting functions and therefore important
    PredictionQueue         prediction_queue_;  /// the predcition queue may not reach ovbersize.

    std::mutex              request_pose_mutex_;
    math::Pose              requset_pose_;
    std::atomic_bool        request_pose_initilization_;
    std::atomic_bool        request_global_initialization_;
    std::atomic_bool        stop_waiting_;

    inline void wait(std::unique_lock<std::mutex> &&lock)
    {
        while(update_queue_.empty()) {
            notify_event_.wait(lock);
            if(stop_waiting_) {
                break;
            }
        }
    }

    inline void processRequests()
    {
        if(request_global_initialization_) {
            request_global_initialization_ = false;
        }
        if(request_pose_initilization_) {
            std::unique_lock<std::mutex> l(request_pose_mutex_);
            request_pose_initilization_ = false;
        }
    }



    inline void filter()
    {
        working_ = true;
        while(!stop_working_) {
            /// 1. process requests which occured
            processRequests();
            /// 2. get a new sensor update from the queue




            /// after resampling reset the weight to 1.0 as suggested by "probabilistic robotics"
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
