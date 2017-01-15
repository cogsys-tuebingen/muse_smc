#ifndef MAP_HPP
#define MAP_HPP

#include <muse_amcl/math/pose.hpp>
#include <muse_amcl/math/point.hpp>
#include <memory>
#include <chrono>

namespace muse_amcl {
class Map {
public:
    typedef std::shared_ptr<Map> Ptr;
    typedef std::shared_ptr<Map const> ConstPtr;

    Map(const std::string &frame) :
        frame_(frame),
        stamp_(ros::Time::now())
    {
    }

    Map(const std::string &frame,
        const ros::Time &stamp) :
        frame_(frame),
        stamp_(stamp)
    {
    }

    virtual ~Map()
    {
    }

    virtual inline bool validate(const math::Pose &p) const
    {
        return true;
    }

    virtual inline math::Point getMin() const
    {
        return math::Point(std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest(),
                           std::numeric_limits<double>::lowest());
    }

    virtual inline math::Point getMax() const
    {
        return math::Point(std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max(),
                           std::numeric_limits<double>::max());
    }

    /**
     * @brief getOrigin return the map origin withi the maps frame.
     * @return
     */
    virtual inline math::Pose getOrigin() const
    {
        return math::Pose(tf::Pose::getIdentity());
    }

    virtual inline bool isLoaded() const
    {
        return true;
    }

    inline std::string getFrame() const
    {
        return frame_;
    }

    inline ros::Time getStamp() const
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
    ros::Time   stamp_;
};
}

#endif /* MAP_HPP */
