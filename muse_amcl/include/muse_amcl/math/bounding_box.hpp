#ifndef BOUNDING_BOX_HPP
#define BOUNDING_BOX_HPP

#include "point.hpp"
#include <array>

namespace muse_amcl {
namespace math {
class BoundingBox {
public:
    using Edge   = std::array<Point,2>;
    using Edges  = std::array<Edge,12>;
    using Points = std::array<Point,8>;

    /**
     * @brief BoundingBox constructor with minimum and maximum.
     * @param min   - the minimum corner
     * @param max   - the maximum corner
     */
    BoundingBox(const Point &min,
                const Point &max) :
        minimum_(min),
        maximum_(max),
        axis_(max - min),
        axis_x_((max - min).x(), 0, 0),
        axis_y_(0, (max - min).y(), 0),
        axis_z_(0, 0, (max - min).z()),
        transform_(tf::createIdentityQuaternion(), min)
    {
    }

    /**
     * @brief Copy constructor allowing to apply a transform in place.
     * @param other - other bounding box
     * @param transform - the transformation to apply
     */
    BoundingBox(const BoundingBox &other,
                const tf::Transform &t) :
        BoundingBox(other)
    {
        transform(t);
    }


    /**
     * @brief Return the minimum corner of the bounding box by reference.
     * @return - reference to the minimum corner
     */
    inline Point &minimum()
    {
        return minimum_;
    }

    /**
     * @brief Return the minimum corner of the bounding box by const reference.
     * @return - const reference to the minimum corner
     */
    inline const Point &minimum() const
    {
        return minimum_;
    }

    /**
     * @brief Return the maximum corner of the bounding box by reference.
     * @return - reference to the maximum corner
     */
    inline Point &maximum()
    {
        return maximum_;
    }

    /**
     * @brief Return the maximum corner of the bounding box by const reference.
     * @return - const reference to the maximum corner
     */
    inline const Point &maximum() const
    {
        return maximum_;
    }

    /**
     * @brief Get the corner points of the bounding box.
     * @return  - the points
     */
    inline Points corners() const
    {
        Points pts;
        corners(pts);
        return pts;
    }

    /**
     * @brief Get the corner points of the bounding box by reference.
     * @param pts - the output points
     */
    inline void corners(Points &pts) const
    {
        pts[0] = minimum_;
        pts[1] = minimum_ + axis_x_;
        pts[2] = minimum_ + axis_x_ + axis_y_;
        pts[3] = minimum_ + axis_y_;

        pts[4] = pts[0] + axis_z_;
        pts[5] = pts[1] + axis_z_;
        pts[6] = pts[2] + axis_z_;
        pts[7] = pts[3] + axis_z_;
    }

    /**
     * @brief Return all edges of the bounding box.
     * @return  - the edges
     */
    inline Edges edges() const
    {
        Edges e;
        edges(e);
        return e;
    }

    /**
     * @brief Return all edges of the bounding box by reference.
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

        /// upwards edges
        edges[4][0] = pts[0];
        edges[4][1] = pts[4];
        edges[5][0] = pts[1];
        edges[5][1] = pts[5];
        edges[6][0] = pts[2];
        edges[6][1] = pts[6];
        edges[7][0] = pts[3];
        edges[7][1] = pts[7];

        /// top layer edges
        edges[8][0]  = pts[4];
        edges[8][1]  = pts[5];
        edges[9][0]  = pts[5];
        edges[9][1]  = pts[6];
        edges[10][0] = pts[6];
        edges[10][1] = pts[7];
        edges[11][0] = pts[7];
        edges[11][1] = pts[4];

    }

    /**
     * @brief Transform the bounding box by mutation.
     * @param transform - transformation to apply
     */
    inline void transform(const tf::Transform &transform)
    {
        minimum_ = transform * minimum_;
        maximum_ = transform * maximum_;
        tf::Transform rotation (transform.getRotation());
        axis_x_  = rotation * axis_x_;
        axis_y_  = rotation * axis_y_;
        axis_z_  = rotation * axis_z_;
        transform_ = transform * transform_;
    }


    /**
     * @brief axisAlignedEnclosing returns the axis aligned bounding box for the bounding box.
     * @return axis aligned bounding box
     */
    inline BoundingBox axisAlignedEnclosing() const
    {
        BoundingBox b;
        axisAlignedEnclosing(b);
        return b;
    }

    /**
     * @brief axisAlignedEnclosing returns the axis aligned bounding box for the bounding box by
     *        reference.
     */
    inline void axisAlignedEnclosing(BoundingBox &bounding) const
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
        bounding = BoundingBox(min, max);
    }

    inline bool axisAlignedIntersection(const BoundingBox &other,
                                        BoundingBox &aintersection)
    {
        BoundingBox aao = other.axisAlignedEnclosing();
        BoundingBox aa = axisAlignedEnclosing();

        Point max = aao.maximum_;
        Point min = aao.minimum_;
        max.setMin(aa.maximum_);
        min.setMax(aa.minimum_);

        aintersection = BoundingBox(min, max);

        tf::Vector3 diagonal = max - min;
        bool valid = true;
        for(std::size_t i = 0 ; i < 3 ; ++i)
            valid &= diagonal[i] >= 0.0;
        return valid;
    }

    inline void axisAlignedUnion(const BoundingBox &other,
                                 BoundingBox &aunion)
    {
        BoundingBox aao = other.axisAlignedEnclosing();
        BoundingBox aa = axisAlignedEnclosing();

        Point max = aao.maximum_;
        Point min = aao.minimum_;
        min.setMin(aa.minimum_);
        max.setMax(aa.maximum_);
        aunion = BoundingBox(min, max);
    }

    inline bool contains(const Point &p) const
    {
        Point p_ = transform_.inverse() * p
               ;
        return p_.x() >= 0.0 && p_.y() >= 0.0 && p_.z() >= 0.0 &&
               p_.x() <= axis_[0] && p_.y() <= axis_[1] && p_.z() <= axis_[2];
    }


    /**
     * Default constructor.
     */
    BoundingBox() = default;
private:

    Point         minimum_;
    Point         maximum_;

    tf::Vector3   axis_;

    tf::Vector3   axis_x_;
    tf::Vector3   axis_y_;
    tf::Vector3   axis_z_;

    tf::Transform transform_;

};
}
}

/**
 * @brief operator * transforms the bounding box and returns a new instance.
 * @param transform - transformation to apply
 */
inline muse_amcl::math::BoundingBox operator * (const tf::Transform &transform ,
                                                const muse_amcl::math::BoundingBox &bb)
{
    return muse_amcl::math::BoundingBox(bb, transform);
}

#endif /* BOUNDING_BOX_HPP */