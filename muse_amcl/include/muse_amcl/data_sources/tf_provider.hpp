#ifndef TF_PROVIDER_HPP
#define TF_PROVIDER_HPP

#include <tf/transform_listener.h>
#include <memory>
#include <mutex>

namespace muse_amcl {
class TFProvider {
public:
    typedef std::shared_ptr<TFProvider> Ptr;

    TFProvider() = default;

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

private:
    std::mutex mutex_;
    tf::TransformListener tf_;
};
}
#endif // TF_PROVIDER_HPP
