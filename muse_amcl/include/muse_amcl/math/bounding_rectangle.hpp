#pragma once

#include "point.hpp"
#include <array>

namespace muse_amcl {
namespace math {
class BoundingRectangle {
public:
    using Edge   = std::array<Point,2>;
    using Edges  = std::array<Edge,4>;
    using Points = std::array<Point,4>;

    BoundingRectangle(const Point &min,
                      const Point &max) :
        minimum_(min),
        maximum_(max),
        axis_x_((max - min).x(), 0, 0),
        axis_y_(0, (max - min).y(), 0)
    {
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
        pts[1] = minimum_ + axis_x_;
        pts[2] = maximum_;
        pts[3] = minimum_ + axis_y_;
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
        axis_x_  = rotation * axis_x_;
        axis_y_  = rotation * axis_y_;
        transform_ = transform;
    }

    /**
     * @brief axisAlignedEnclosing returns the axis aligned bounding rectangle
     * for the bounding rectangle.
     * @return axis aligned bounding rectangle
     */
    inline BoundingRectangle axisAlignedEnclosing() const
    {
        BoundingRectangle b;
        axisAlignedEnclosing(b);
        return b;
    }

    /**
     * @brief axisAlignedEnclosing returns the axis aligned bounding rectangle
     *        for the bounding rectangle by reference.
     */
    inline void axisAlignedEnclosing(BoundingRectangle &bounding) const
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
        bounding = BoundingRectangle(min, max);
    }

private:
    /**
     * Default constructor.
     */
    BoundingRectangle() = default;


    Point         minimum_;
    Point         maximum_;

    tf::Vector3   axis_x_;
    tf::Vector3   axis_y_;

    tf::Transform transform_;

};
}
}

/**
 * @brief operator * transforms the bounding rectangle and returns a new instance.
 * @param transform - transformation to apply
 */
inline muse_amcl::math::BoundingRectangle operator * (const tf::Transform &transform ,
                                                      const muse_amcl::math::BoundingRectangle bb)
{
    return muse_amcl::math::BoundingRectangle(bb, transform);
}

