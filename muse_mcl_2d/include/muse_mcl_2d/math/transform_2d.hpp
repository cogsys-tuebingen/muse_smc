#ifndef TRANSFORM_2D_HPP
#define TRANSFORM_2D_HPP

#include <muse_mcl_2d/math/vector_2d.hpp>

namespace muse_mcl_2d {
class Transform2D {
public:
    inline Transform2D() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(0.0)
    {
    }

    inline Transform2D(const double x,
                const double y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2D(const Vector2D &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2D(const double yaw,
                const double x,
                const double y) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2D(const Vector2D &translation,
                const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Vector2D operator * (const Vector2D &v)
    {
        return Vector2D(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                        sin_ * v.x() + cos_ * v.y() + translation_.y());

    }

    inline Transform2D operator * (const Transform2D &other)
    {
        Transform2D t;
        t.sin_ = sin_ * other.cos_ + other.sin_ * cos_;
        t.cos_ = cos_ * other.cos_ - other.sin_ * sin_;
        t.yaw_ = std::acos(t.cos_);
        t.translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        t.translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        return t;
    }


    inline Transform2D & operator *= (const Transform2D &other)
    {

        const double s = sin_ * other.cos_ + other.sin_ * cos_;
        const double c = cos_ = cos_ * other.cos_ - other.sin_ * sin_;
        translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        yaw_ = std::acos(c);
        sin_ = s;
        cos_ = c;
        return *this;
    }

    inline double & tx()
    {
        return translation_.x();
    }

    inline double tx() const
    {
        return translation_.x();
    }

    inline double & ty()
    {
        return translation_.y();
    }

    inline double ty() const
    {
        return translation_.y();
    }

    inline Vector2D & translation()
    {
        return translation_;
    }

    inline Vector2D const & translation() const
    {
        return translation_;
    }

    inline double & yaw()
    {
        return yaw_;
    }

    inline double yaw() const
    {
        return yaw_;
    }

private:
    Vector2D translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
};
}


#endif // TRANSFORM_2D_HPP
