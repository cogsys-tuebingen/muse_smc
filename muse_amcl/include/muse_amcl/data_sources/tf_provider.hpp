#ifndef TF_PROVIDER_HPP
#define TF_PROVIDER_HPP

#include <tf/transform_listener.h>
#include <memory>
#include <muse_amcl/utils/delegate.hpp>
#include <mutex>

#include <muse_amcl/utils/logger.hpp>

namespace muse_amcl {
class TFProvider {
public:
    class LockedTFProvider {
    public:
        LockedTFProvider(std::unique_lock<std::mutex> &&lock,
                         TFProvider &provider) :
            lock_(std::move(lock)),
            provider_(provider)
        {
        }

        LockedTFProvider(LockedTFProvider &&other) :
            lock_(std::move(other.lock_)),
            provider_(other.provider_)
        {
        }

        LockedTFProvider(const LockedTFProvider &other) = delete;

        virtual ~LockedTFProvider()
        {
        }

        inline bool lookupTransform(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::StampedTransform &transform)
        {
            return provider_.lookupTransformImpl(target_frame, source_frame, time, transform);
        }

        inline bool lookupTransform(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::StampedTransform &transform,
                                    const ros::Duration  &timeout)
        {
            return provider_.lookupTransformImpl(target_frame, source_frame, time, transform, timeout);
        }


        inline bool lookupTransform(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::Transform        &transform)
        {
            return provider_.lookupTransform(target_frame, source_frame, time, transform);
        }

        inline bool lookupTransform(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::Transform        &transform,
                                    const ros::Duration  &timeout)
        {
            return provider_.lookupTransformImpl(target_frame, source_frame, time, transform, timeout);
        }

        inline bool canTransform(const std::string &target_frame,
                                 const std::string &source_frame,
                                 const ros::Time   &time)
        {
            return provider_.tf_.canTransform(target_frame, source_frame, time);
        }

        inline bool waitForTransform(const std::string &target_frame,
                                     const std::string &source_frame,
                                     const ros::Time &time,
                                     const ros::Duration &timeout)
        {
            return provider_.tf_.waitForTransform(target_frame, source_frame, time, timeout);
        }

        inline void getFrameStrings(std::vector<std::string> &frames)
        {
            provider_.tf_.getFrameStrings(frames);
        }
    protected:
        std::unique_lock<std::mutex>  lock_;
        TFProvider                   &provider_;
    };

    friend class LockedTFProvider;
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
        return lookupTransformImpl(target_frame, source_frame, time, transform);
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::StampedTransform &transform,
                                const ros::Duration  &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        return lookupTransformImpl(target_frame, source_frame, time, transform, timeout);
    }


    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::Transform        &transform)
    {
        std::unique_lock<std::mutex> l(mutex_);
        return lookupTransform(target_frame, source_frame, time, transform);
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::Transform        &transform,
                                const ros::Duration  &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        return lookupTransformImpl(target_frame, source_frame, time, transform, timeout);
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

    inline LockedTFProvider getLockedTFProvider()
    {
        std::unique_lock<std::mutex> lock;
        return LockedTFProvider(std::move(lock), *this);
    }


protected:
    std::mutex mutex_;
    tf::TransformListener tf_;

    inline bool lookupTransformImpl(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::StampedTransform &transform)
    {
        try  {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
        } catch (const tf::TransformException &e) {
            Logger::getLogger().error(e.what(), "TFProvider");
            return false;
        }
        return true;
    }

    inline bool lookupTransformImpl(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::StampedTransform &transform,
                                    const ros::Duration  &timeout)
    {
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout)) {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
            return true;
        }
        return false;
    }


    inline bool lookupTransformImpl(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::Transform        &transform)
    {
        tf::StampedTransform stamped;
        try  {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = stamped;
        } catch (const tf::TransformException &e) {
            Logger::getLogger().error(e.what(), "TFProvider");
            return false;
        }
        return true;
    }

    inline bool lookupTransformImpl(const std::string    &target_frame,
                                    const std::string    &source_frame,
                                    const ros::Time      &time,
                                    tf::Transform        &transform,
                                    const ros::Duration  &timeout)
    {
        tf::StampedTransform stamped;
        std::string error;
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout, ros::Duration(0.01), &error)) {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = stamped;
            return true;
        } else {
            Logger::getLogger().error(error, "TFProvider");
        }
        return false;
    }
};
}
#endif // TF_PROVIDER_HPP
