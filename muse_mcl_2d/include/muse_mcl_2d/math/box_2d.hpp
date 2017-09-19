#ifndef BOX_2D_HPP
#define BOX_2D_HPP

#include <muse_mcl_2d/math/line_2d.hpp>

#include <limits>
#include <set>

namespace muse_mcl_2d {
class Box2D
{
public:
    using point_set_t    = std::set<Point2D>;
    using coefficients_t = std::array<double, 2>;

    Box2D() :
        min_{std::numeric_limits<double>::lowest(),
             std::numeric_limits<double>::lowest()},
        max_{std::numeric_limits<double>::max(),
             std::numeric_limits<double>::max()}
    {
    }

    Box2D(const double min_x, const double min_y,
          const double max_x, const double max_y) :
        min_{min_x, min_y},
        max_{max_x, max_y}
    {
    }

    Box2D(const Point2D &min,
          const Point2D &max) :
        min_(min),
        max_(max)
    {
    }

    inline void setMin(const Point2D &min)
    {
        min_ = min;
    }

    inline void setMax(const Point2D &max)
    {
        max_ = max;
    }

    inline Point2D const & getMin() const
    {
        return min_;
    }

    inline Point2D const & getMax() const
    {
        return max_;
    }

    inline bool intersects(const Line2D &line) const
    {
        //// LIANG BARSKY
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        double t0 = 0.0;
        double t1 = 1.0;

        auto clip = [] (const double p, const double q,
                        double &t0, double &t1)
        {
            if(p == 0 && q < 0)
                return false;

            const double r = q / p;
            if(p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if(p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d.x(), -(min_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(d.x(), (max_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(-d.y(), -(min_.y()-p0.y()), t0, t1))
                return false;

        if(!clip(d.y(), (max_.y()-p0.y()), t0, t1))
                return false;
        return true;
    }

    inline bool intersection(const Line2D &line,
                             Line2D &clipped)
    {
        const auto p0 = line[0];
        const auto p1 = line[1];
        const auto d = p1 - p0;

        double t0 = 0.0;
        double t1 = 1.0;

        auto clip = [] (const double p, const double q,
                        double &t0, double &t1)
        {
            if(p == 0 && q < 0)
                return false;

            const double r = q / p;
            if(p < 0) {
                if(r > t1)
                    return false;
                t0 = r > t0 ? r : t0;
            }
            if(p > 0) {
                if(r < t0)
                    return false;
                t1 = r < t1 ? r : t1;
            }
            return true;
        };

        if(!clip(-d.x(), -(min_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(d.x(), (max_.x()-p0.x()), t0, t1))
                return false;

        if(!clip(-d.y(), -(min_.y()-p0.y()), t0, t1))
                return false;

        if(!clip(d.y(), (max_.y()-p0.y()), t0, t1))
                return false;

        clipped[0].x() = p0.x() + t0*d.x();
        clipped[0].y() = p0.y() + t0*d.y();
        clipped[1].x() = p0.x() + t1*d.x();
        clipped[1].y() = p0.y() + t1*d.y();

        return true;
    }

private:
    Point2D min_;
    Point2D max_;

}__attribute__ ((aligned (32)));
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::Box2D &b)
{
    out << "[" << b.getMin() << "," << b.getMax() << "]";
    return out;
}

#endif // BOX_2D_HPP
