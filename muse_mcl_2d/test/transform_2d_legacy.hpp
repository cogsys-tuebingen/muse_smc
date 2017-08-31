#ifndef TRANSFORM_2D_LEGACY_HPP
#define TRANSFORM_2D_LEGACY_HPP

#ifndef TRANSFORM_2D_HPP
#define TRANSFORM_2D_HPP

#include <muse_mcl_2d/math/vector_2d.hpp>

#include <muse_smc/utility/stamped.hpp>
#include <muse_smc/math/angle.hpp>

namespace muse_mcl_2d {
class Transform2DLegacy {
public:
    inline Transform2DLegacy() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const double x,
                       const double y) :
        translation_(x, y),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const Vector2D &translation) :
        translation_(translation),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    inline Transform2DLegacy(const double x,
                       const double y,
                       const double yaw) :
        translation_(x, y),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2DLegacy(const Vector2D &translation,
                       const double yaw) :
        translation_(translation),
        yaw_(yaw),
        sin_(std::sin(yaw_)),
        cos_(std::cos(yaw_))
    {
    }

    inline Transform2DLegacy(const Transform2DLegacy &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2DLegacy(Transform2DLegacy &&other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Vector2D operator * (const Vector2D &v) const
    {
        return Vector2D(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                        sin_ * v.x() + cos_ * v.y() + translation_.y());

    }

    inline Transform2DLegacy operator * (const Transform2DLegacy &other) const
    {
        Transform2DLegacy t;
        t.setYaw(muse_smc::math::angle::normalize(yaw_ + other.yaw_));
        t.translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        t.translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        return t;
    }


    inline Transform2DLegacy & operator *= (const Transform2DLegacy &other)
    {
        translation_.x() = cos_ * other.translation_.x() - sin_ * other.translation_.y() + translation_.x();
        translation_.y() = sin_ * other.translation_.x() + cos_ * other.translation_.y() + translation_.y();
        setYaw(muse_smc::math::angle::normalize(yaw_ + other.yaw_));
        return *this;
    }

    inline Transform2DLegacy& operator = (const Transform2DLegacy &other)
    {
        if(&other != this) {
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
            translation_ = other.translation_;
        }
        return *this;
    }

    inline Transform2DLegacy& operator = (Transform2DLegacy &&other)
    {
        if(&other != this) {
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
            translation_ = other.translation_;
        }
        return *this;
    }

    inline Transform2DLegacy inverse() const
    {
        Transform2DLegacy t;
        t.setYaw(-yaw_);
        t.translation_ = -(t * translation_);
        return t;
    }

    inline Transform2DLegacy operator -() const
    {
        return inverse();
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

    inline double sin() const
    {
        return sin_;
    }

    inline double cos() const
    {
        return cos_;
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

    inline Transform2DLegacy interpolate(const Transform2DLegacy &other,
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
        return Transform2DLegacy(translation, yaw);
    }

private:
    Vector2D translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
};

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::Transform2DLegacy &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

using StampedTransform2D = muse_smc::Stamped<Transform2DLegacy>;
}


#endif // TRANSFORM_2D_HPP


#endif // TRANSFORM_2D_LEGACY_HPP
