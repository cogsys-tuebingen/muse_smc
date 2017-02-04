#ifndef LASER_SCAN_2D_HPP
#define LASER_SCAN_2D_HPP

#include <muse_amcl/math/point.hpp>
#include <muse_amcl/data_types/data.hpp>

namespace muse_amcl {
class LaserScan2D : public muse_amcl::Data
{
public:
    struct Ray {
        const double angle;
        const double range;
        math::Point  point;

        Ray(const double angle,
            const double range) :
            angle(angle),
            range(range),
            point(math::Point(std::cos(angle) * range,
                              std::sin(angle) * range))
        {
        }
    };

    using Rays = std::vector<Ray>;

    LaserScan2D(const std::string &frame) :
        Data(frame)
    {
    }

    LaserScan2D(const std::string &frame,
                const TimeFrame &time_frame) :
        Data(frame, time_frame)
    {
    }

    inline void setRangeInterval(const double range_min,
                                 const double range_max)
    {
        range_min_ = range_min;
        range_max_ = range_max;
    }

    inline void setAngleInteraval(const double angle_min,
                                  const double angle_max)
    {
        angle_min_ = angle_min;
        angle_max_ = angle_max;
    }

    inline double getRangeMin() const
    {
        return range_min_;
    }

    inline double getRangeMax() const
    {
        return range_max_;
    }

    inline double getAngleMin() const
    {
        return angle_min_;
    }

    inline double getAngleMax() const
    {
        return angle_max_;
    }

    inline const Rays& getRays() const
    {
        return rays_;
    }

    inline Rays& getRays()
    {
        return rays_;
    }

private:
    Rays rays_;         /// only valid rays shall be contained here
    double range_min_;
    double range_max_;
    double angle_min_;
    double angle_max_;
};
}

#endif // LASER_SCAN_2D_HPP
