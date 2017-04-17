#ifndef TRANSFORM_PUBLISHER_HPP
#define TRANSFORM_PUBLISHER_HPP

#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/time.h>

namespace muse_amcl {
class TransformPublisher {
public:
    using Ptr = std::shared_ptr<TransformPublisher>;

    TransformPublisher(const double rate) :
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

    inline void setTransform(const tf::StampedTransform &tf)
    {
        std::unique_lock<std::mutex> l(tf_mutex_);
        tf_ = tf;
        wait_for_transform_ = false;
    }

    inline void resetTransform()
    {
        wait_for_transform_ = true;
    }


private:
    std::atomic_bool         running_;
    std::atomic_bool         stop_;
    std::thread              worker_thread_;
    std::mutex               tf_mutex_;
    tf::StampedTransform     tf_;
    std::atomic_bool         wait_for_transform_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Rate                tf_rate_;

    void loop()
    {
        running_ = true;
        while(!stop_) {
            if(!wait_for_transform_) {
                std::unique_lock<std::mutex> l(tf_mutex_);
                tf_broadcaster_.sendTransform(tf_);
            }
            tf_rate_.sleep();
        }
        running_ = false;
    }
};


}

#endif // TRANSFORM_PUBLISHER_HPP
