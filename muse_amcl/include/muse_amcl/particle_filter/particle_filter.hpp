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
    std::priority_queue<Update::Ptr, std::vector<Update::Ptr>, Update::Greater>;
    using PredictionQueue =
    std::priority_queue<Prediction::Ptr, std::vector<Prediction::Ptr>, Prediction::Greater>;

    ParticleFilter();
    virtual ~ParticleFilter();

    void setup(ros::NodeHandle &nh_private, const TFProvider::Ptr &tf_provider);

    /// uniform pose sampling
    void setUniformSampling(const UniformSampling::Ptr &sampling_uniform);
    UniformSampling::Ptr getUniformSampling() const;

    /// normal pose sampling
    void setNormalsampling(NormalSampling::Ptr &sampling_normal_pose);
    NormalSampling::Ptr getNormalSampling() const;

    /// resampling
    void setResampling(Resampling::Ptr &resampling);
    Resampling::Ptr getResampling() const;

    /// insert new predictions
    void addPrediction(Prediction::Ptr &prediction);

    void addUpdate(Update::Ptr &update);

    void requestPoseInitialization(const math::Pose &pose, const math::Covariance &covariance);

    void requestGlobalInitialization();

    void start();

    void end();


protected:
    const std::string       name_;

    ros::Publisher          pub_poses_;
    ros::Time               pub_poses_last_time_;
    ros::Time               pub_tf_last_time_;

    TFProvider::Ptr         tf_provider_;
    ros::Time               particle_set_stamp_;
    ParticleSet::Ptr        particle_set_;

    UniformSampling::Ptr    sampling_uniform_;
    NormalSampling::Ptr     sampling_normal_pose_;
    Resampling::Ptr         resampling_;

    std::mutex              worker_thread_mutex_;
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

    void processRequests();
    bool processPredictions(const ros::Time &until);
    void publishPoses(const bool force = false);
    void publishTF();
    void loop();

    std::string privateParameter(const std::string &name)
    {
        return name_ + "/" + name;
    }


};
}

#endif // PARTICLE_FILTER_HPP
