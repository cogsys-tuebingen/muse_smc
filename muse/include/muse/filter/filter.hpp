#ifndef FILTER_HPP
#define FILTER_HPP

#include <muse/resampling/resampling.hpp>
#include <muse_mcl/sampling/sampling_normal.hpp>
#include <muse_mcl/sampling/sampling_uniform.hpp>

#include <muse_mcl/particle_filter/particle_set.hpp>
#include <muse_mcl/prediction/prediction.hpp>
#include <muse_mcl/update/update.hpp>
#include <muse_mcl/utility/csv_logger.hpp>
#include <muse_mcl/utility/filterstate_publisher.hpp>
#include <muse_mcl/utility/dotty.hpp>

#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <condition_variable>
#include <map>

namespace muse {
template<typename sample_t, typename update_t, typename prediction_t, typename ... setup_args_t>
class ParticleFilter
{
public:
    using Ptr = std::shared_ptr<ParticleFilter>;
    using update_queue_t =
    std::priority_queue<typename update_t::Ptr,
                        std::deque<typename update_t::Ptr>,
                        typename update_t::Greater>;

    using prediction_queue_t =
    std::priority_queue<typename prediction_t::Ptr,
                        std::deque<typename prediction_t::Ptr>,
                        typename prediction_t::Greater>;

    ParticleFilter();
    virtual ~ParticleFilter();

    void setup(const setup_args_t& ... args);

    /// uniform pose sampling
    void setUniformSampling(const SamplingUniform::Ptr &sampling_uniform);
    SamplingUniform::Ptr getUniformSampling() const;

    /// normal pose sampling
    void setNormalsampling(SamplingNormal::Ptr &sampling_normal_pose);
    SamplingNormal::Ptr getNormalSampling() const;

    /// resampling
    void setResampling(Resampling::Ptr &resampling);
    Resampling::Ptr getResampling() const;

    /// density estimation



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
    TFPublisher::Ptr          tf_publisher_;

    FilterStatePublisher::Ptr filter_state_publisher_;
    Dotty::Ptr                dotty_;

    ros::Time                particle_set_stamp_;
    ParticleSet::Ptr         particle_set_;
    math::Pose               particle_set_mean_;

    SamplingUniform::Ptr     sampling_uniform_;
    SamplingNormal::Ptr      sampling_normal_pose_;
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

    std::map<ModelUpdate*, double> abs_motion_integrals_linear_update_;  /// track motion integral for each update model
    std::map<ModelUpdate*, double> abs_motion_integrals_angular_update_; /// track motion integral for each update model

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

#endif // FILTER_HPP
