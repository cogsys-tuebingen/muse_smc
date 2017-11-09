#ifndef LASER_SCAN_2D_HPP
#define LASER_SCAN_2D_HPP

#include <muse_smc/data/data.hpp>

#include <cslibs_math_2d/types/point.hpp>
#include <cslibs_time/time_frame.hpp>

#include <limits>
#include <vector>

namespace muse_mcl_2d_laser {
class LaserScan2D : public muse_smc::Data
{
public:
    using point_t       = cslibs_math_2d::Point2d;
    using time_frame_t  = cslibs_time::TimeFrame;
    using interval_t    = std::array<double, 2>;

    struct Ray {
        const double  angle;
        const double  range;
        const point_t point;

        inline Ray(const double angle,
                   const double range) :
            angle(angle),
            range(range),
            point(point_t(std::cos(angle) * range,
                          std::sin(angle) * range))
        {
        }

        inline Ray(const point_t &pt) :
                   angle(cslibs_math_2d::angle(pt)),
                   range(pt.length()),
                   point(pt)
        {
        }

        inline Ray() :
            angle(0.0),
            range(0.0),
            point(point_t())
        {
        }

        inline Ray(const Ray &other) :
            angle(other.angle),
            range(other.range),
            point(other.point)
        {
        }

        inline Ray(Ray &&other) :
            angle(other.angle),
            range(other.range),
            point(std::move(other.point))
        {
        }

        inline bool valid() const
        {
            return range != 0.0;
        }

    };

    using Ptr              = std::shared_ptr<LaserScan2D>;
    using rays_t           = std::vector<Ray>;
    using const_iterator_t = rays_t::const_iterator;

    LaserScan2D(const std::string &frame,
                const time_frame_t &time_frame) :
        Data(frame, time_frame),
        linear_interval_{0.0, std::numeric_limits<double>::max()},
        angular_interval_{-M_PI, M_PI}
    {
    }

    LaserScan2D(const std::string &frame,
                const time_frame_t &time_frame,
                const interval_t &linear_interval,
                const interval_t &angular_interval) :
        Data(frame, time_frame),
        linear_interval_(linear_interval),
        angular_interval_(angular_interval)
    {
    }

    inline void setLinearInterval(const double min,
                                  const double max)
    {
        linear_interval_[0] = min;
        linear_interval_[1] = max;
    }

    inline void setLinearInterval(const interval_t &interval)
    {
        linear_interval_ = interval;
    }

    inline void setAngularInterval(const double min,
                                   const double max)
    {
        angular_interval_[0] = min;
        angular_interval_[1] = max;
    }

    inline void setAngularInterval(const interval_t &interval)
    {
        angular_interval_ = interval;
    }

    inline double getLinearMin() const
    {
        return linear_interval_[0];
    }

    inline double getLinearMax() const
    {
        return linear_interval_[1];
    }

    inline double getAngularMin() const
    {
        return angular_interval_[0];
    }

    inline double getAngularMax() const
    {
        return angular_interval_[1];
    }

    inline void insert(const double angle,
                       const double range)
    {
        rays_.emplace_back(Ray(angle, range));
    }

    inline void insert(const point_t &pt)
    {
        rays_.emplace_back(Ray(pt));
    }

    inline void insertInvalid()
    {
        rays_.emplace_back(Ray());
    }

    inline const_iterator_t begin() const
    {
        return rays_.begin();
    }

    inline const_iterator_t end() const
    {
        return rays_.end();
    }

    inline const rays_t& getRays() const
    {
        return rays_;
    }

private:
    rays_t     rays_;         /// only valid rays shall be contained here
    interval_t linear_interval_;
    interval_t angular_interval_;
};
}

#endif // LASER_SCAN_2D_HPP
