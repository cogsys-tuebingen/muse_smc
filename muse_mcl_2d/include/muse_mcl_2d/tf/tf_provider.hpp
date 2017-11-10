#ifndef TF_PROVIDER_HPP
#define TF_PROVIDER_HPP

#include <muse_smc/utility/delegate.hpp>
#include <cslibs_time/stamped.hpp>

#include <cslibs_math_2d/linear/transform.hpp>
#include <tf/transform_listener.h>
#include <memory>
#include <mutex>

namespace muse_mcl_2d {
class TFProvider {
public:
    using Ptr = std::shared_ptr<TFProvider>;
    using stamped_t = cslibs_time::Stamped<cslibs_math_2d::Transform2d>;

    inline TFProvider()
    {
    }

    virtual ~TFProvider()
    {
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                stamped_t            &transform)
    {
        tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame,time, tf_transform)) {
            convert(tf_transform, transform.data());
            transform.stamp() = cslibs_time::Time(time.toNSec());
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string           &target_frame,
                                const std::string           &source_frame,
                                const ros::Time             &time,
                                stamped_t    &transform,
                                const ros::Duration         &timeout)
    {
        tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            convert(tf_transform, transform.data());
            transform.stamp() = cslibs_time::Time(time.toNSec());
            return true;
        }
        return false;
    }


    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                cslibs_math_2d::Transform2d    &transform)
    {
        tf::Transform tf_transform;
        if(lookupTransform(target_frame, source_frame, time, tf_transform)) {
            convert(tf_transform, transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                cslibs_math_2d::Transform2d    &transform,
                                const ros::Duration  &timeout)
    {
        tf::Transform tf_transform;
        if(lookupTransform(target_frame,
                           source_frame,
                           time,
                           tf_transform,
                           timeout)) {
            convert(tf_transform, transform);
            return true;
        }
        return false;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::StampedTransform &transform)
    {
        std::unique_lock<std::mutex> l(mutex_);
        std::string error;
        if(tf_.canTransform(target_frame, source_frame, time, &error)) {
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

    inline void convert(const tf::Transform &src, cslibs_math_2d::Transform2d &dst)
    {
        const tf::Vector3 &origin = src.getOrigin();
        const tf::Quaternion &rotation = src.getRotation();
        dst.setFrom(origin.x(), origin.y(), tf::getYaw(rotation));
    }
};
}
#endif // TF_PROVIDER_HPP
