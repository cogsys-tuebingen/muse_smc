#ifndef BOUNDING_RECTANGLE_HPP
#define BOUNDING_RECTANGLE_HPP

#include "point.hpp"
#include <array>

namespace muse_amcl {
namespace math {
class BoundingRectangle {
public:
    using Edge   = std::array<Point,2>;
    using Edges  = std::array<Edge, 4>;
    using Points = std::array<Point,4>;

    BoundingRectangle(const Point &min,
                      const Point &max) :
        minimum_(min),
        maximum_(max),
        axis_1st_((max - min).x(), 0, 0),
        axis_2nd_(0, (max - min).y(), 0)
    {
        assert(min.z() == 0);
        assert(max.z() == 0);
    }

    BoundingRectangle(const Point &min,
                      const tf::Vector3 &axis_1st,
                      const tf::Vector3 &axis_2nd) :
        minimum_(min),
        maximum_(min + axis_1st + axis_2nd),
        axis_1st_(axis_1st),
        axis_2nd_(axis_2nd)
    {
        assert(axis_1st.dot(axis_2nd) == 0);
    }

    /**
     * @brief Copy constructor allowing to apply a transform in place.
     * @param other - other bounding rectangle
     * @param transform - the transformation to apply
     */
    BoundingRectangle(const BoundingRectangle &other,
                      const tf::Transform &t) :
        BoundingRectangle(other)
    {
        transform(t);
    }

    /**
     * @brief Return the minimum corner of the bounding rectangle by reference.
     * @return - reference to the minimum corner
     */
    inline Point &minimum()
    {
        return minimum_;
    }

    /**
     * @brief Return the minimum corner of the bounding rectangle by const reference.
     * @return - const reference to the minimum corner
     */
    inline const Point &minimum() const
    {
        return minimum_;
    }

    /**
     * @brief Return the maximum corner of the bounding rectangle by reference.
     * @return - reference to the maximum corner
     */
    inline Point &maximum()
    {
        return maximum_;
    }

    /**
     * @brief Return the maximum corner of the bounding rectangle by const reference.
     * @return - const reference to the maximum corner
     */
    inline const Point &maximum() const
    {
        return maximum_;
    }

    /**
     * @brief Get the corner points of the bounding rectangle.
     * @return  - the points
     */
    inline Points corners() const
    {
        Points pts;
        corners(pts);
        return pts;
    }

    /**
     * @brief Get the corner points of the bounding rectangle by reference.
     * @param pts - the output points
     */
    inline void corners(Points &pts) const
    {
        pts[0] = minimum_;
        pts[1] = minimum_ + axis_1st_;
        pts[2] = maximum_;
        pts[3] = minimum_ + axis_2nd_;
    }

    /**
     * @brief Return all edges of the bounding rectangle.
     * @return  - the edges
     */
    inline Edges edges() const
    {
        Edges e;
        edges(e);
        return e;
    }

    /**
     * @brief Return all edges of the bounding rectangle by reference.
     * @param edges - output reference for the edges
     */
    inline void edges(Edges &edges) const
    {
        Points pts = corners();
        /// bottom plane edges
        edges[0][0] = pts[0];
        edges[0][1] = pts[1];
        edges[1][0] = pts[1];
        edges[1][1] = pts[2];
        edges[2][0] = pts[2];
        edges[2][1] = pts[3];
        edges[3][0] = pts[3];
        edges[3][1] = pts[0];
    }

    /**
     * @brief Transform the bounding rectangle by mutation.
     * @param transform - transformation to apply
     */
    inline void transform(const tf::Transform &transform)
    {
        minimum_ = transform * minimum_;
        maximum_ = transform * maximum_;
        tf::Transform rotation (transform.getRotation());
        axis_1st_  = rotation * axis_1st_;
        axis_2nd_  = rotation * axis_2nd_;
        transform_ = transform;
    }

    /**
     * @brief axisAlignedEnclosingXY returns the axis aligned enclosing
     *        rectangle projected to the xy plane.
     * @return  the enclosing rectangle in the xy plane.
     */
    inline BoundingRectangle axisAlignedEnclosingXY() const
    {
        BoundingRectangle b;
        axisAlignedEnclosingXY(b);
        return b;
    }

    /**
     * @brief axisAlignedEnclosingXY returns the axis aligned enclosing
     *        rectangle projected to the xy plane by reference.
     * @param bounding - reference to return result to
     */
    inline void axisAlignedEnclosingXY(BoundingRectangle &bounding) const
    {
        using limits = std::numeric_limits<tfScalar>;
        Point min(limits::max(), limits::max(), limits::max());
        Point max(limits::lowest(), limits::lowest(), limits::lowest());
        Points pts;
        corners(pts);
        for(const Point &p : pts) {
            min.setMin(p);
            max.setMax(p);
        }
        tf::Vector3 diagonal = max - min;
        diagonal[2] = 0;
        tf::Vector3 axis_1st = diagonal;
        tf::Vector3 axis_2nd = diagonal;
        axis_1st[1] = 0;
        axis_2nd[0] = 0;
        bounding = BoundingRectangle(min, axis_1st, axis_2nd);
    }

    /**
     * @brief axisAlignedEnclosingXZ returns the axis aligned enclosing
     *        rectangle projected to the xy plane.
     * @return  the enclosing rectangle in the xz plane.
     */
    inline BoundingRectangle axisAlignedEnclosingXZ() const
    {
        BoundingRectangle b;
        axisAlignedEnclosingXZ(b);
        return b;
    }

    /**
     * @brief axisAlignedEnclosingXZ returns the axis aligned enclosing
     *        rectangle projected to the xz plane by reference.
     * @param bounding - reference to return result to
     */
    inline void axisAlignedEnclosingXZ(BoundingRectangle &bounding) const
    {
        using limits = std::numeric_limits<tfScalar>;
        Point min(limits::max(), limits::max(), limits::max());
        Point max(limits::lowest(), limits::lowest(), limits::lowest());
        Points pts;
        corners(pts);
        for(const Point &p : pts) {
            min.setMin(p);
            max.setMax(p);
        }
        tf::Vector3 diagonal = max - min;
        diagonal[1] = 0;
        tf::Vector3 axis_1st = diagonal;
        tf::Vector3 axis_2nd = diagonal;
        axis_1st[2] = 0;
        axis_2nd[0] = 0;
        bounding = BoundingRectangle(min, axis_1st, axis_2nd);
    }

    /**
     * @brief axisAlignedEnclosingYZ returns the axis aligned enclosing
     *        rectangle projected to the yz plane.
     * @return  the enclosing rectangle in the xz plane.
     */
    inline BoundingRectangle axisAlignedEnclosingYZ() const
    {
        BoundingRectangle b;
        axisAlignedEnclosingYZ(b);
        return b;
    }

    /**
     * @brief axisAlignedEnclosingYZ returns the axis aligned enclosing
     *        rectangle projected to the yz plane by reference.
     * @param bounding - reference to return result to
     */
    inline void axisAlignedEnclosingYZ(BoundingRectangle &bounding) const
    {
        using limits = std::numeric_limits<tfScalar>;
        Point min(limits::max(), limits::max(), limits::max());
        Point max(limits::lowest(), limits::lowest(), limits::lowest());
        Points pts;
        corners(pts);
        for(const Point &p : pts) {
            min.setMin(p);
            max.setMax(p);
        }
        tf::Vector3 diagonal = max - min;
        diagonal[0] = 0;
        tf::Vector3 axis_1st = diagonal;
        tf::Vector3 axis_2nd = diagonal;
        axis_1st[2] = 0;
        axis_2nd[1] = 0;
        bounding = BoundingRectangle(min, axis_1st, axis_2nd);
    }

    inline bool axisAlignedIntersection(const BoundingRectangle &other,
                                        BoundingRectangle &aintersection)
    {
        Point max = other.maximum_;
        Point min = other.minimum_;
        max.setMin(maximum_);
        min.setMax(minimum_);

        aintersection = BoundingRectangle(min, max);

        tf::Vector3 diagonal = max - min;
        bool valid = true;
        for(std::size_t i = 0 ; i < 3 ; ++i)
            valid &= diagonal[i] >= 0.0;
        return valid;
    }

    inline void axisAlignedUnion(const BoundingRectangle &other,
                                 BoundingRectangle &aunion)
    {
        Point max = other.maximum_;
        Point min = other.minimum_;
        min.setMin(minimum_);
        max.setMax(maximum_);
        aunion = BoundingRectangle(min, max);
    }


    /**
     * Default constructor.
     */
    BoundingRectangle() = default;
private:


    Point         minimum_;
    Point         maximum_;

    tf::Vector3   axis_1st_;
    tf::Vector3   axis_2nd_;

    tf::Transform transform_;

};
}
}

/**
 * @brief operator * transforms the bounding rectangle and returns a new instance.
 * @param transform - transformation to apply
 */
inline muse_amcl::math::BoundingRectangle operator * (const tf::Transform &transform ,
                                                      const muse_amcl::math::BoundingRectangle &bb)
{
    return muse_amcl::math::BoundingRectangle(bb, transform);
}


#endif /* BOUNDING_RECTANGLE_HPP */
