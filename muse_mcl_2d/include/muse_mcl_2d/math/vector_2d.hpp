#ifndef VECTOR_2D_HPP
#define VECTOR_2D_HPP

#include <cmath>
#include <eigen3/Eigen/Core>

namespace muse_mcl_2d {
class Vector2D {
public:
    inline Vector2D() :
        x_(0.0),
        y_(0.0)
    {
    }

    inline Vector2D(const double x,
                    const double y) :
        x_(x),
        y_(y)
    {
    }

    inline Vector2D(const Vector2D &other) :
        x_(other.x_),
        y_(other.y_)
    {
    }

    inline Vector2D(Vector2D &&other) :
        x_(other.x_),
        y_(other.y_)
    {
    }

    inline Vector2D operator * (const double d) const
    {
        return Vector2D(x_ * d, y_ * d);
    }

    inline Vector2D operator / (const double d) const
    {
        return Vector2D(x_ / d, y_ / d);
    }

    inline Vector2D operator + (const Vector2D &other) const
    {
        return Vector2D(x_ + other.x_, y_ + other.y_);
    }

    inline Vector2D operator - (const Vector2D &other) const
    {
        return Vector2D(x_ - other.x_, y_ - other.y_);
    }

    inline double dot (const Vector2D &other) const
    {
        return x_ * other.x_ + y_ * other.y_;
    }

    inline double length () const
    {
        return std::hypot(x_, y_);
    }

    inline double angle() const
    {
        return std::atan2(y_, x_);
    }

    inline double & x()
    {
        return x_;
    }

    inline double & y()
    {
        return y_;
    }

    inline double const & x() const
    {
        return x_;
    }

    inline double const & y() const
    {
        return y_;
    }

    inline Vector2D & operator += (const Vector2D &other)
    {
        x_ += other.x_;
        y_ += other.y_;
        return *this;
    }

    inline Vector2D & operator -= (const Vector2D &other)
    {
        x_ -= other.x_;
        y_ -= other.y_;
        return *this;
    }

    inline Vector2D & operator *= (const double d)
    {
        x_ *= d;
        y_ *= d;
        return *this;
    }

    inline Vector2D & operator /= (const double d)
    {
        x_ /= d;
        y_ /= d;
        return *this;
    }

    inline Vector2D& operator = (const Vector2D &other)
    {
        if(&other != this) {
            x_ = other.x_;
            y_ = other.y_;
        }
        return *this;
    }

    inline Vector2D& operator = (Vector2D &&other)
    {
        if(&other != this) {
            x_ = other.x_;
            y_ = other.y_;
        }
        return *this;
    }

    inline Eigen::Vector2d toEigen() const
    {
        return Eigen::Vector2d(x_, y_);
    }

    inline Vector2D min(const Vector2D &other) const
    {
        return Vector2D(std::min(x_, other.x_),
                        std::min(y_, other.y_));
    }

    inline Vector2D max(const Vector2D &other) const
    {
        return Vector2D(std::max(x_, other.x_),
                        std::max(y_, other.y_));
    }

private:
    double x_;
    double y_;
};
}
#endif // VECTOR_2D_HPP
