#ifndef TRANSFORM_2D_HPP
#define TRANSFORM_2D_HPP

#include <muse_mcl_2d/math/vector_2d.hpp>
#include <muse_smc/utility/stamped.hpp>
#include <muse_smc/math/angle.hpp>

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

    inline Vector2D operator * (const Vector2D &v) const
    {
        return Vector2D(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                        sin_ * v.x() + cos_ * v.y() + translation_.y());

    }

    inline Transform2D operator * (const Transform2D &other) const
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

    inline Transform2D inverse() const
    {
        Transform2D t;
        t.yaw_ = -yaw_;
        t.sin_ = -sin_;
        t.cos_ = cos_;
        t.translation_ = t * t.translation_;
        return t;
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

    inline void setYaw(const double yaw)
    {
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }


    inline double yaw() const
    {
        return yaw_;
    }

    inline Eigen::Vector3d toEigen() const
    {
        return Eigen::Vector3d(translation_.x(), translation_.y(), yaw_);
    }

    inline void setFrom(const Eigen::Vector3d &eigen)
    {
        translation_.x() = eigen(0);
        translation_.y() = eigen(1);
        yaw_ = eigen(2);
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline void setFrom(const double x,
                        const double y,
                        const double yaw)
    {
        translation_.x() = x;
        translation_.y() = y;
        yaw_ = yaw;
        sin_ = std::sin(yaw_);
        cos_ = std::cos(yaw_);
    }

    inline Transform2D interpolate(const Transform2D &other,
                                   const double ratio) const
    {
        assert(ratio  >= 0.0);
        assert(ratio <= 1.0);
        if(ratio == 0.0) {
            return *this;
        }
        if(ratio == 1.0) {
            return other;
        }

        const  double ratio_inverse = 1.0 - ratio;
        const  Vector2D translation = translation_ * ratio_inverse + other.translation_ * ratio;
        const  double   yaw = muse_smc::math::angle::normalize(yaw_ * ratio_inverse + other.yaw_ * ratio);
        return Transform2D(translation, yaw);
    }

private:
    Vector2D translation_;
    double   yaw_;
    double   sin_;
    double   cos_;


};

using StampedTransform2D = muse_smc::Stamped<Transform2D>;

}


#endif // TRANSFORM_2D_HPP
