#ifndef TRANSFORM_PUBLISHER_ANCHORED_HPP
#define TRANSFORM_PUBLISHER_ANCHORED_HPP


#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <ros/time.h>

namespace muse_mcl {
/**
 * @brief The TransformPublisherAnchored class can be used to published
 *        transformations consisting of a relative transformation in a moving
 *        coordinate frame and a anchor transformation in a fixed frame.
 *        More precisely, relative motion of the base link in the odometry frame
 *        is calculated and combined with the fixed anchor transformation in the
 *        world frame.
 *
 */
class TransformPublisher {
public:
    using Ptr = std::shared_ptr<TransformPublisher>;

    /**
     * @brief TransformPublisherAnchored constructor.
     * @param rate          - the publication rate
     * @param odom_frame    - the odometry frame
     * @param base_frame    - the base frame
     * @param world_frame   - the world fram
     */
    TransformPublisher(const double rate,
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
        wait_for_transform_(true),
        tf_rate_(rate)
    {
    }

    virtual ~TransformPublisher()
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

    inline void setTransform(const tf::StampedTransform &w_t_b)
    {
        std::unique_lock<std::mutex> l(tf_mutex_);
        w_T_b_ = w_t_b;
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
    tf::StampedTransform     w_T_o_;
    tf::StampedTransform     w_T_b_;
    std::atomic_bool         wait_for_transform_;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener    tf_listener_;
    ros::Rate                tf_rate_;

    void loop()
    {
        running_ = true;
        while(!stop_) {
            if(!wait_for_transform_) {
                tf::StampedTransform b_T_o;
                if(tf_listener_.waitForTransform(base_frame_, odom_frame_, w_T_b_.stamp_, timeout_)) {
                    tf_listener_.lookupTransform(base_frame_, odom_frame_, w_T_b_.stamp_, b_T_o);

                    std::unique_lock<std::mutex> l(tf_mutex_);
                    w_T_o_ = tf::StampedTransform(static_cast<tf::Transform>(w_T_b_) * b_T_o,
                                                  w_T_b_.stamp_, world_frame_, odom_frame_);
                    w_T_o_.stamp_ = ros::Time::now();
                    tf_broadcaster_.sendTransform(w_T_o_);
                }
            }
            tf_rate_.sleep();
        }
        running_ = false;
    }
};


}

#endif // TRANSFORM_PUBLISHER_ANCHORED_HPP
