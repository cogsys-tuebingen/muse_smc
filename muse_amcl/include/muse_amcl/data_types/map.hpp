#pragma once

#include <muse_amcl/math/pose.hpp>
#include <muse_amcl/math/point.hpp>
#include <memory>
#include <chrono>

namespace muse_amcl {
class Map {
public:
    typedef std::shared_ptr<Map> Ptr;
    typedef std::shared_ptr<Map const> ConstPtr;

    Map(const std::string &_frame) :
        frame_(_frame),
        stamp_(std::chrono::system_clock::now())
    {
    }

    Map(const std::string &_frame,
        const std::chrono::time_point<std::chrono::system_clock> &_stamp) :
        frame_(_frame),
        stamp_(_stamp)
    {
    }

    virtual ~Map()
    {
    }

    virtual inline bool valid(const math::Pose &_p) const
    {
        return true;
    }

    virtual inline math::Point min() const
    {
        return math::Point(std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest());
    }

    virtual inline math::Point max() const
    {
        return math::Point(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
    }

    virtual inline bool isLoaded() const
    {
        return true;
    }

    inline std::string frame() const
    {
        return frame_;
    }

    inline std::chrono::time_point<std::chrono::system_clock> stamp() const
    {
        return stamp_;
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
    Map() = delete;

    std::string frame_;
    std::chrono::time_point<std::chrono::system_clock> stamp_;
};
}
