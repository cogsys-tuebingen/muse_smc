#pragma once

#include <tf/tf.h>
#include <memory>

namespace muse_amcl {
class Map {
public:
    typedef std::shared_ptr<Map> Ptr;
    typedef std::shared_ptr<Map const> ConstPtr;

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

    virtual inline bool isLoaded() const
    {
        return true;
    }

    Map(const std::string &_frame) :
        frame_(_frame)
    {
    }

    inline std::string frame()
    {
        return frame_;
    }

    template<typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template<typename T>
    T const * as() const
    {
        return dynamic_cast<const T*>(this);
    }

protected:
    Map(){}

    std::string frame_;
};
}
