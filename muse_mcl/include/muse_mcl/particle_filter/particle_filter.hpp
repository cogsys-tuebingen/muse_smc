#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP

#include <muse_mcl/data_sources/tf_provider.hpp>
#include <muse_mcl/particle_filter/particle_set.hpp>
#include <muse_mcl/particle_filter/resampling.hpp>
#include <muse_mcl/particle_filter/sampling_normal.hpp>
#include <muse_mcl/particle_filter/sampling_uniform.hpp>
#include <muse_mcl/particle_filter/prediction.hpp>
#include <muse_mcl/particle_filter/update.hpp>
#include <muse_mcl/utils/csv_logger.hpp>
#include <muse_mcl/utils/transform_publisher.hpp>
#include <muse_mcl/utils/filterstate_publisher.hpp>
#include <muse_mcl/utils/dotty.hpp>

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <map>

namespace muse_mcl {
class ParticleFilter {
public:
    using Ptr = std::shared_ptr<ParticleFilter>;
    using UpdateQueue     =
    std::priority_queue<Update::Ptr, std::deque<Update::Ptr>, Update::Greater>;
    using PredictionQueue =
    std::priority_queue<Prediction::Ptr, std::deque<Prediction::Ptr>, Prediction::Greater>;

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
    enum PredictionOutcome {NO_MOTION, MOTION, RETRY};

    const std::string        name_;
    TFProvider::Ptr          tf_provider_;


    tf::StampedTransform      tf_latest_w_T_b_;
    TransformPublisher::Ptr   tf_publisher_;

    FilterStatePublisher::Ptr filter_state_publisher_;
    Dotty::Ptr                dotty_;

    ros::Time                particle_set_stamp_;
    ParticleSet::Ptr         particle_set_;
    math::Pose               particle_set_mean_;

    UniformSampling::Ptr     sampling_uniform_;
    NormalSampling::Ptr      sampling_normal_pose_;
    Resampling::Ptr          resampling_;
    std::size_t              resampling_cycle_;

    std::mutex               worker_thread_mutex_;
    std::thread              worker_thread_;

    std::atomic_bool         working_;
    std::atomic_bool         stop_working_;
    std::condition_variable  notify_event_;
    mutable std::mutex       notify_mutex_;
    std::condition_variable  notify_prediction_;
    mutable std::mutex       notify_prediction_mutex_;

    //// ------------------ parameters ----------------------///
    double                   resampling_threshold_linear_;
    double                   resampling_threshold_angular_;
    std::size_t              update_cycle_;
    ros::Duration            pub_poses_delay_;
    std::string              world_frame_;
    std::string              odom_frame_;
    std::string              base_frame_;
    bool                     integrate_all_measurement_;

    //// ------------------ working members ----------------///
    mutable std::mutex       update_queue_mutex_;
    mutable std::mutex       prediction_queue_mutex_;
    UpdateQueue              update_queue_;                             /// this is for the weighting functions and therefore important
    PredictionQueue          prediction_queue_;                         /// the predcition queue may not reach ovbersize.

    std::map<UpdateModel*, double> abs_motion_integrals_linear_update_;  /// track motion integral for each update model
    std::map<UpdateModel*, double> abs_motion_integrals_angular_update_; /// track motion integral for each update model

    double                   abs_motion_integral_linear_resampling_;     /// Motion integral for resampling threshold
    double                   abs_motion_integral_angular_resampling_;    /// Motion integral for resampling threshold

    //// ------------------ requests -----------------------///
    std::mutex               request_pose_mutex_;
    math::Pose               initialization_pose_;
    math::Covariance         initialization_covariance_;
    std::atomic_bool         request_pose_initilization_;
    std::atomic_bool         request_global_initialization_;

    //// ------------------ logger -------------------------///
    FilterStateLoggerDefault::Ptr filter_state_logger_;

    void processRequests();
    void processPredictions(const ros::Time &until,
                            double &abs_motion_integral_linear_update,
                            double &abs_motion_integral_angular_update);
    void publishPoses();
    void publishTF();
    void loop();
    void tryToResample();

    std::string privateParameter(const std::string &name) const;
    void saveFilterState() const;
    bool updatesQueued() const;
    Update::Ptr getUpdate();
    void queueUpdate(const Update::Ptr &update);
    void applyUpdate(Update::Ptr &update);
};
}

#endif // PARTICLE_FILTER_HPP
