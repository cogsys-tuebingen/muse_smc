#ifndef LASER_SCAN_2D_HPP
#define LASER_SCAN_2D_HPP

#include <muse_amcl/math/point.hpp>
#include <muse_amcl/data_types/data.hpp>

namespace muse_amcl {
class LaserScan2D : muse_amcl::Data
{
public:
    using Points = std::vector<math::Point>;
    using Ranges = std::vector<float>;
    using Angles = std::vector<float>;

    LaserScan2D(const std::string &frame) :
        Data(frame)
    {
    }

    LaserScan2D(const std::string &frame,
                const TimeFrame &time_frame) :
        Data(frame, time_frame)
    {
    }

    inline void setCartesian(const std::vector<math::Point> &points)
    {
        points_ = points;
    }

    inline void setPolar(const std::vector<float> &ranges,
                         const std::vector<float> &angles)
    {
        ranges_ = ranges;
        angles_ = angles;
    }

    inline const Points& getPoints() const
    {
        return points_;
    }

    inline const Ranges& getRanges() const
    {
        return ranges_;
    }

    inline const Angles& getAngles() const
    {
        return angles_;
    }

private:
    Points points_;
    Ranges ranges_;
    Angles angles_;
};
}

#endif // LASER_SCAN_2D_HPP
