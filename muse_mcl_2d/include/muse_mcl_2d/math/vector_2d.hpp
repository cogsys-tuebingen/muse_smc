#ifndef VECTOR_2D_HPP
#define VECTOR_2D_HPP

#include <cmath>

namespace muse_mcl_2d {
class Vector2D {
public:
    Vector2D() :
        x_(0.0),
        y_(0.0)
    {
    }

    Vector2D(const double x,
            const double y) :
        x_(x),
        y_(y)
    {
    }

    inline Vector2D operator * (const double d)
    {
        return Vector2D(x_ * d, y_ * d);
    }

    inline Vector2D operator / (const double d)
    {
        return Vector2D(x_ / d, y_ / d);
    }

    inline Vector2D operator + (const Vector2D &other)
    {
        return Vector2D(x_ + other.x_, y_ + other.y_);
    }

    inline Vector2D operator - (const Vector2D &other)
    {
        return Vector2D(x_ - other.x_, y_ - other.y_);
    }

    inline double dot (const Vector2D &other)
    {
        return x_ * other.x_ + y_ * other.y_;
    }

    inline double len () const
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

private:
    double x_;
    double y_;
};
}
#endif // VECTOR_2D_HPP
