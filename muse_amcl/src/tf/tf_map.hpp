#ifndef TF_MAP_HPP
#define TF_MAP_HPP

#include <tf/tf.h>
#include <memory>

namespace muse_amcl {
namespace conversion {
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
}
}

#endif /* TF_MAP_HPP */
