#pragma once

#include <tf/tf.h>
#include <memory>

namespace muse_amcl {
namespace maps {
struct Map {
    typedef std::shared_ptr<Map> Ptr;
    virtual inline bool valid(const tf::Pose &_p) const
    {
        return true;
    }

    virtual inline tf::Point min() const
    {
        return tf::Point(std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest(),
                         std::numeric_limits<double>::lowest());
    }

    virtual inline tf::Point max() const
    {
        return tf::Point(std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max(),
                         std::numeric_limits<double>::max());
    }

    Map(const std::string &_frame) :
        frame(_frame)
    {
    }

    const std::string frame;
};
}
}
