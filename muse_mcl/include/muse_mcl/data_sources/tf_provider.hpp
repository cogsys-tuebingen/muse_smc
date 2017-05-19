#ifndef TF_PROVIDER_HPP
#define TF_PROVIDER_HPP

#include <tf/transform_listener.h>
#include <memory>
#include <muse_mcl/utils/delegate.hpp>
#include <mutex>

#include <muse_mcl/utils/logger.hpp>

namespace muse_mcl {
class TFProvider {
public:
    typedef std::shared_ptr<TFProvider> Ptr;

    TFProvider()
    {
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::StampedTransform &transform)
    {
        std::unique_lock<std::mutex> l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
            Logger::getLogger().error(error, "TFProvider");
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        } else {
            return false;
        }
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::StampedTransform &transform,
                                const ros::Duration  &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout)) {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        }
        return false;
    }


    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::Transform        &transform)
    {
        std::unique_lock<std::mutex> l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
            Logger::getLogger().error(error, "TFProvider");
            tf::StampedTransform stamped;
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = static_cast<tf::Transform>(stamped);
            return true;
        } else {
            return false;
        }
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::Transform        &transform,
                                const ros::Duration  &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        tf::StampedTransform stamped;
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout)) {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = static_cast<tf::Transform>(stamped);
            return true;
        }
        return false;
    }

    inline bool canTransform(const std::string &target_frame,
                             const std::string &source_frame,
                             const ros::Time   &time)
    {
        std::unique_lock<std::mutex> l(mutex_);
        return tf_.canTransform(target_frame, source_frame, time);
    }

    inline bool waitForTransform(const std::string &target_frame,
                                 const std::string &source_frame,
                                 const ros::Time &time,
                                 const ros::Duration &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        return tf_.waitForTransform(target_frame, source_frame, time, timeout);
    }

    inline void getFrameStrings(std::vector<std::string> &frames)
    {
        std::unique_lock<std::mutex> l(mutex_);
        tf_.getFrameStrings(frames);
    }

protected:
    std::mutex mutex_;
    tf::TransformListener tf_;
};
}
#endif // TF_PROVIDER_HPP
