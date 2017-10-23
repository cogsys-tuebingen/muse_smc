#ifndef TRANSFORM_PUBLISHER_ANCHORED_HPP
#define TRANSFORM_PUBLISHER_ANCHORED_HPP


#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <cslibs_math_2d/types/transform.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>
#include <cslibs_math_2d/conversion/tf.hpp>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/time.h>

namespace muse_mcl_2d {
/**
 * @brief The TransformPublisherAnchored class can be used to published
 *        transformations consisting of a relative transformation in a moving
 *        coordinate frame and a anchor transformation in a fixed frame.
 *        More precisely, relative motion of the base link in the odometry frame
 *        is calculated and combined with the fixed anchor transformation in the
 *        world frame.
 *
 */
class TFPublisher {
public:
    using Ptr = std::shared_ptr<TFPublisher>;
    using stamped_t = muse_smc::Stamped<cslibs_math_2d::Transform2d>;

    /**
     * @brief TransformPublisherAnchored constructor.
     * @param rate          - the publication rate
     * @param odom_frame    - the odometry frame
     * @param base_frame    - the base frame
     * @param world_frame   - the world fram
     */
    inline TFPublisher(const double rate,
                       const std::string &odom_frame,
                       const std::string &base_frame,
                       const std::string &world_frame,
                       const double timeout = 0.1) :
        odom_frame_(odom_frame),
        base_frame_(base_frame),
        world_frame_(world_frame),
        timeout_(timeout),
        running_(false),
        stop_(false),
        w_T_b_(cslibs_math_2d::Transform2d::identity(), muse_smc::Time(ros::Time::now().toNSec())),
        wait_for_transform_(true),
        tf_rate_(rate)
    {
    }

    inline virtual ~TFPublisher()
    {
        end();
    }

    inline void start()
    {
        if(running_)
            return;

        worker_thread_ = std::thread([this]{loop();});
        worker_thread_.detach();
    }

    inline void end()
    {
        if(!running_)
            return;

        stop_ = true;
        if(worker_thread_.joinable())
            worker_thread_.join();
    }

    inline void setTransform(const stamped_t &w_t_b)
    {
        std::unique_lock<std::mutex> l(tf_mutex_);
        w_T_b_ = w_t_b;

        cslibs_math_2d::Transform2d b_T_o = cslibs_math_2d::Transform2d::identity();
        if(tf_listener_.lookupTransform(base_frame_, odom_frame_, ros::Time(w_T_b_.stamp().seconds()), b_T_o, timeout_)) {
            cslibs_math_2d::Transform2d w_T_o = w_T_b_.data() * b_T_o;
            w_T_o_ = tf::StampedTransform(cslibs_math_2d::conversion::from(w_T_o), ros::Time(w_T_b_.stamp().seconds()), world_frame_, odom_frame_);

            tf_time_of_transform_ = w_T_o_.stamp_;
        }

        wait_for_transform_ = false;
    }

    inline void resetTransform()
    {
        wait_for_transform_ = true;
    }


private:
    const std::string        odom_frame_;
    const std::string        base_frame_;
    const std::string        world_frame_;
    const ros::Duration      timeout_;

    std::atomic_bool         running_;
    std::atomic_bool         stop_;
    std::thread              worker_thread_;
    std::mutex               tf_mutex_;

    tf::TransformBroadcaster tf_broadcaster_;
    TFProvider               tf_listener_;

    tf::StampedTransform     w_T_o_;
    stamped_t                w_T_b_;
    std::atomic_bool         wait_for_transform_;
    ros::Rate                tf_rate_;
    ros::Time                tf_time_of_transform_;

    inline void loop()
    {
        running_ = true;
        while(!stop_) {
            if(!wait_for_transform_) {
                std::unique_lock<std::mutex> l(tf_mutex_);
                w_T_o_.stamp_ = ros::Time::now();
                tf_broadcaster_.sendTransform(w_T_o_);
            }
            tf_rate_.sleep();
        }
        running_ = false;
    }
};


}

#endif // TRANSFORM_PUBLISHER_ANCHORED_HPP
