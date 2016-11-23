#pragma once

#include <eigen3/Eigen/Core>
#include <tf/tf.h>

namespace muse_amcl {
namespace math {
/**
 * @brief The Point class is an adapter class for Eigen and tf types.
 */
class Point {
public:
    using Point3d = Eigen::Vector3d;
    using Point2d = Eigen::Vector2d;

    /**
     * @brief Point default constructor.
     */
    Point() :
        point_(0.0,0.0,0.0)
    {
    }

    /**
     * @brief Point constructor for tf data types.
     * @param point
     */
    Point(const tf::Point &point) :
        point_(point)
    {
    }

    /**
     * @brief Point created from 2D Eigen data type.
     * @param point - 2D Eigen vector
     */
    Point(const Point2d &point) :
        point_(point(0),point(1),0)
    {
    }

    /**
     * @brief Point created from 3D Eigen data type.
     * @param point - 3D Eigen vector
     */
    Point(const Point3d &point) :
        point_(point(0),point(1),point(2))
    {
    }

    /**
     * @brief tf returns a reference to the encapsuled tf point.
     * @return
     */
    inline tf::Point & tf()
    {
        return point_;
    }

    /**
     * @brief tf returns a const reference to the encapsuled tf point.
     * @return
     */
    inline const tf::Point & tf() const
    {
        return point_;
    }

    /**
     * @brief eigen2D returns a 2D Eigen-based point.
     * @return
     */
    inline Point2d eigen2D() const
    {
        return Point2d(point_.x(), point_.y());
    }

    /**
     * @brief eigen3D returns a 3D Eigen-based point.
     * @return
     */
    inline Point3d eigen3D() const
    {
        return Point3d(point_.x(), point_.y(), point_.z());
    }

private:
    tf::Point point_;

};
}
}
