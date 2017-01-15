#ifndef TF_PROVIDER_HPP
#define TF_PROVIDER_HPP

#include <tf/transform_listener.h>
#include <memory>
#include <mutex>

namespace muse_amcl {
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
        try  {
            tf_.lookupTransform(target_frame, source_frame, time, transform);
        } catch (const tf::TransformException &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
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
        tf::StampedTransform stamped;
        try  {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = stamped;
        } catch (const tf::TransformException &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    inline bool lookupTransform(const std::string    &target_frame,
                                const std::string    &source_frame,
                                const ros::Time      &time,
                                tf::Transform        &transform,
                                const ros::Duration  &timeout)
    {
        std::unique_lock<std::mutex> l(mutex_);
        tf::StampedTransform stamped;
        std::string error;
        if(tf_.waitForTransform(target_frame, source_frame, time, timeout, ros::Duration(0.01), &error)) {
            tf_.lookupTransform(target_frame, source_frame, time, stamped);
            transform = stamped;
            return true;
        } else {
            std::cerr << error << std::endl;
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

    inline void getFrameStrings(std::vector<std::string> &frames)
    {
        std::unique_lock<std::mutex> l(mutex_);
        tf_.getFrameStrings(frames);
    }


private:
    std::mutex mutex_;
    tf::TransformListener tf_;
};
}
#endif // TF_PROVIDER_HPP
