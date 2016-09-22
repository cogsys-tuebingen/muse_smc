#ifndef TFPROVIDER_H
#define TFPROVIDER_H

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <memory>

namespace muse {
namespace transforms {
/**
 * @brief Multikey object to identify a transformation.
 */
struct TransformKey {

    TransformKey(const std::string &target,
                 const std::string &source) :
        target(target),
        source(source)
    {
    }

    std::string target;
    std::string source;
};

struct less {
    bool operator()(const TransformKey &a,
                    const TransformKey &b)
    {
        return a.target < b.target && a.source < b.source;
    }
};

typedef std::map<TransformKey, tf::StampedTransform, less> TransformMap;

/**
 * @brief The TFProvider class keeps track of all tranformations when any sensor
 *        gets an update.
 *        Read only regarding the tf tree.
 */
class TFProvider
{
public:
    typedef std::shared_ptr<TFProvider> Ptr;

    TFProvider();

    bool updateTransform(const std::string    &target,
                         const std::string    &source,
                         const ros::Time      &time,
                         const ros::Duration  &tolerance);
    bool updateTransform(const std::string    &target,
                         const std::string    &source,
                         const ros::Time      &time,
                         const ros::Duration  &tolerance,
                         tf::StampedTransform &transform);

    tf::StampedTransform getTransformation(const std::string &target,
                                           const std::string &source);

    void getTransformation(const std::string  &target,
                           const std::string  &source,
                           tf::StampedTransform &transform);

private:
    tf::TransformListener tfl_;
    TransformMap          tfm_;
};
}
}

#endif // TFPROVIDER_H
