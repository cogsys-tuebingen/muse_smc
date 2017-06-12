#ifndef LASER_SCAN_2D_HPP
#define LASER_SCAN_2D_HPP

#include <muse_mcl/math/point.hpp>
#include <muse_mcl/data_types/data.hpp>

#include <limits>

namespace muse_mcl {
class LaserScan2D : public muse_mcl::Data
{
public:
    struct Ray {
        const double       angle_;
        const double       range_;
        const math::Point  point_;
        const bool         valid_;

        Ray(const double angle,
            const double range) :
            angle_(angle),
            range_(range),
            point_(math::Point(std::cos(angle) * range_,
                               std::sin(angle) * range_)),
            valid_(true)
        {
        }

        Ray(const math::Point &pt) :
            angle_(std::atan2(pt.y(), pt.x())),
            range_(std::hypot(pt.y(), pt.x())),
            point_(pt),
            valid_(true)
        {
        }

        Ray() :
            angle_(std::numeric_limits<double>::infinity()),
            range_(std::numeric_limits<double>::infinity()),
            point_(math::Point()),
            valid_(false)
        {
        }
    };

    using Ptr  = std::shared_ptr<LaserScan2D>;
    using Rays = std::vector<Ray>;

    LaserScan2D(const std::string &frame) :
        Data(frame),
        range_min_(0.0),
        range_max_(std::numeric_limits<double>::max()),
        angle_min_(-M_PI),
        angle_max_(+M_PI)
    {
    }

    LaserScan2D(const std::string &frame,
                const TimeFrame &time_frame) :
        Data(frame, time_frame),
        range_min_(0.0),
        range_max_(std::numeric_limits<double>::max()),
        angle_min_(-M_PI),
        angle_max_(+M_PI)
    {
    }

    inline void setRangeInterval(const double range_min,
                                 const double range_max)
    {
        range_min_ = range_min;
        range_max_ = range_max;
    }

    inline void setAngleInterval(const double angle_min,
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
