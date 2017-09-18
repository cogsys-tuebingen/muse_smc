#ifndef BOX_2D_HPP
#define BOX_2D_HPP

#include <muse_mcl_2d/math/point_2d.hpp>

#include <limits>
#include <set>

namespace muse_mcl_2d {
class Box2D
{
public:
    using point_set_t = std::set<Point2D>;

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

    inline bool intersects() const
    {

    }

    inline void intersection(const Point2D &start,
                             const Point2D &end,
                             point_set_t   &result)
    {

    }

    inline bool intersectionFromWithin(const Point2D &start,
                                       const Point2D &end,
                                       Point2D &result) const
    {

    }




private:
    Point2D min_;
    Point2D max_;


}__attribute__ ((aligned (32)));
}


#endif // BOX_2D_HPP
