#pragma once

#include <eigen3/Eigen/Core>
#include <tf/tf.h>

namespace muse_amcl {
namespace math {
/**
 * @brief The Point class is an adapter class for Eigen and tf types.
 */
class Point : public tf::Point {
public:
    using Point3d = Eigen::Vector3d;
    using Point2d = Eigen::Vector2d;
    using Point3f = Eigen::Matrix<tfScalar, 3, 1>;
    using Point2f = Eigen::Matrix<tfScalar, 2, 1>;

    Point(const double x = .0,
          const double y = .0,
          const double z = .0) :
        tf::Point(x, y, z)
    {
    }

    Point(const tf::Point &other) :
        tf::Point(other)
    {
    }

    /**
     * @brief Point created from 2D Eigen data type.
     * @param point - 2D Eigen vector
     */
    Point(const Point2d &point) :
        tf::Point(point(0),point(1),0)
    {
    }

    /**
     * @brief Point created from 3D Eigen data type.
     * @param point - 3D Eigen vector
     */
    Point(const Point3d &point) :
        tf::Point(point(0),point(1),point(2))
    {
    }

    inline const tfScalar &x() const
    {
        return tf::Point::x();
    }

    inline const tfScalar &y() const
    {
        return tf::Point::y();
    }

    inline const tfScalar &z() const
    {
        return tf::Point::z();
    }

    inline tfScalar &x()
    {
        return m_floats[0];
    }

    inline tfScalar &y()
    {
        return m_floats[1];
    }

    inline tfScalar &z()
    {
        return m_floats[2];
    }

    /**
     * @brief eigen2D returns a 2D Eigen-based point.
     * @return
     */
    inline Point2f eigen2D()
    {
        return Eigen::Map<Point2f>(m_floats);
    }

    /**
     * @brief eigen3D returns a 3D Eigen-based point.
     * @return
     */
    inline Point3f eigen3D()
    {
        return Eigen::Map<Point3f>(m_floats);
    }


    /**
     * @brief Return the element-wise minimum.
     * @param other - another point
     * @return  the minimum
     */
    inline Point cwiseMin(const Point &other)
    {
        Point p(*this);
        p.setMin(other);
        return p;
    }

    /**
     * @brief Return the element-wise minimum.
     * @param other - another point
     * @param max - reference to the variable the value should be return to.
     */
    inline void cwiseMin(const Point &other,
                                Point &min)
    {
        min = *this;
        min.setMin(other);
    }

    /**
     * @brief Return the element-wise maximum.
     * @param other - another point
     * @return  the maximumg
     */
    inline Point cwiseMax(const Point &other)
    {
        Point p(*this);
        p.setMax(other);
        return p;
    }

    /**
     * @brief Return the element-wise maximum.
     * @param other - another point
     * @param max - reference to the variable the value should be return to.
     */
    inline void cwiseMax(const Point &other,
                         Point &max)
    {
        max = *this;
        max.setMax(other);
    }

    inline Point operator + (const tf::Vector3 &other) const
    {
        return Point(m_floats[0] + other.m_floats[0],
                     m_floats[1] + other.m_floats[1],
                     m_floats[2] + other.m_floats[2]);
    }

    inline Point operator + (const Point &other) const
    {
        return Point(m_floats[0] + other.m_floats[0],
                     m_floats[1] + other.m_floats[1],
                     m_floats[2] + other.m_floats[2]);
    }

    inline Point operator * (const tf::Transform &transform) const
    {
        return Point(transform * (tf::Point)(*this));
    }

};
}
}
