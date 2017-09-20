#ifndef TRANSFORM_2D_HPP
#define TRANSFORM_2D_HPP

#include <muse_mcl_2d/math/vector_2d.hpp>

#include <muse_smc/utility/stamped.hpp>
#include <muse_smc/math/angle.hpp>

namespace muse_mcl_2d {
namespace math {
class Transform2D {
public:
    inline Transform2D() :
        translation_(0.0, 0.0),
        yaw_(0.0),
        sin_(0.0),
        cos_(1.0)
    {
    }

    static inline Transform2D identity()
    {
        return Transform2D(0.0, 0.0);
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

    inline Transform2D(const double yaw) :
        Transform2D(0.0, 0.0, yaw)
    {
    }

    inline Transform2D(const double x,
                       const double y,
                       const double yaw) :
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

    inline Transform2D(const Transform2D &other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Transform2D(Transform2D &&other) :
        translation_(other.translation_),
        yaw_(other.yaw_),
        sin_(other.sin_),
        cos_(other.cos_)
    {
    }

    inline Vector2D operator * (const Vector2D &v) const
    {
        return yaw_ == 0.0 ? v + translation_
                           : Vector2D(cos_ * v.x() - sin_ * v.y() + translation_.x(),
                                      sin_ * v.x() + cos_ * v.y() + translation_.y());
    }

    inline Transform2D operator * (const Transform2D &other) const
    {
        return yaw_ == 0.0 ? Transform2D(other.translation_ + translation_,
                                         other.yaw_,
                                         other.sin_,
                                         other.cos_)
                           : other.yaw_ == 0.0 ?              Transform2D((*this) * other.translation_,
                                                                          yaw_,
                                                                          sin_,
                                                                          cos_)
                                                            : Transform2D ((*this) * other.translation_,
                                                                           muse_smc::math::angle::normalize(yaw_ + other.yaw_),
                                                                           sin_ * other.cos_ + cos_ * other.sin_,
                                                                           cos_ * other.cos_ - sin_ * other.sin_);
    }


    inline Transform2D & operator *= (const Transform2D &other)
    {
        if(yaw_ == 0.0) {
            translation_ += other.translation_;
            yaw_ = other.yaw_;
            sin_ = other.sin_;
            cos_ = other.cos_;
        } else if(other.yaw_ == 0.0) {
            translation_ = (*this) * other.translation_;
        } else {
            translation_ = (*this) * other.translation_;
            yaw_ = muse_smc::math::angle::normalize(yaw_ + other.yaw_);
            const double s = sin_ * other.cos_ + cos_ * other.sin_;
            const double c = cos_ * other.cos_ - sin_ * other.sin_;
            sin_ = s;
            cos_ = c;
        }
        return *this;
    }

    inline Transform2D& operator = (const Transform2D &other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform2D& operator = (Transform2D &&other)
    {
        yaw_ = other.yaw_;
        sin_ = other.sin_;
        cos_ = other.cos_;
        translation_ = other.translation_;
        return *this;
    }

    inline Transform2D inverse() const
    {
        return Transform2D(Vector2D(-cos_ * translation_.x() - sin_ * translation_.y(),
                                    sin_ * translation_.x() - cos_ * translation_.y()),
                           -yaw_,
                           -sin_,
                           cos_);
    }

    inline Transform2D operator -() const
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
        setYaw(eigen(2));
    }

    inline void setFrom(const double x,
                        const double y,
                        const double yaw)
    {
        translation_.x() = x;
        translation_.y() = y;
        setYaw(yaw);
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
    inline Transform2D(const Vector2D &translation,
                       const double yaw,
                       const double sin,
                       const double cos) :
        translation_(translation),
        yaw_(yaw),
        sin_(sin),
        cos_(cos)
    {
    }

    Vector2D translation_;
    double   yaw_;
    double   sin_;
    double   cos_;
} __attribute__ ((aligned (64)));

using StampedTransform2D = muse_smc::Stamped<Transform2D>;
}
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::math::Transform2D &t)
{
    out << "[" << t.tx() << "," << t.ty() << "," << t.yaw() << "]";
    return out;
}

#endif // TRANSFORM_2D_HPP
