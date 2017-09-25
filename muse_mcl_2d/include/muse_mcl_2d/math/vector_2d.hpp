#ifndef VECTOR_2D_HPP
#define VECTOR_2D_HPP

#include <cmath>
#include <eigen3/Eigen/Core>
#include <smmintrin.h>

namespace muse_mcl_2d {
namespace math {
class Vector2D {
public:
    inline Vector2D() :
    #if __SSE2__
        v_(_mm_setzero_pd())
  #else
        v_{0.0,0.0}
  #endif

    {
    }

    inline Vector2D(const double x,
                    const double y) :
    #if __SSE2__
        v_(_mm_set_pd(x,y))
  #else
        v_{x,y}
  #endif
    {
    }

#if __SSE2__
    inline Vector2D(const __m128d &v) :
        v_(v)
    {
    }
#else
    return Vector2D(v_[0] * d,v_[1] * d);
#endif


    inline Vector2D(const Vector2D &other) :
        v_(other.v_)
    {
    }

    inline Vector2D(Vector2D &&other) :
        v_(other.v_)
    {
    }

    inline Vector2D operator * (const double d) const
    {
#if __SSE2__
        return Vector2D(_mm_mul_pd(v_, _mm_set1_pd(d)));
#else
        return Vector2D(v_[0] * d,v_[1] * d);
#endif
    }

    inline Vector2D operator / (const double d) const
    {
#if __SSE2__
        return Vector2D(_mm_div_pd(v_, _mm_set1_pd(d)));
#else
        return Vector2D(v_[0] / d,v_[1] / d);
#endif
    }

    inline Vector2D operator + (const Vector2D &other) const
    {
#if __SSE2__
        return Vector2D(_mm_add_pd(v_, other.v_));
#else
        return Vector2D(v_[0] + other.v_[0],v_[1] + other.v_[1]);
#endif
    }

    inline Vector2D operator - (const Vector2D &other) const
    {
#if __SSE2__
        return Vector2D(_mm_add_pd(v_, other.v_));
#else
        return Vector2D(v_[0] - other.v_[0],v_[1] - other.v_[1]);
#endif
    }

    inline double dot (const Vector2D &other) const
    {
#if __SSE2__
        return (_mm_dp_pd(v_, other.v_, 0x31))[0];
#else
        return v_[0] * other.v_[0] +v_[1] * other.v_[1];
#endif
    }

    inline double length () const
    {
#if __SSE2__
        return (_mm_sqrt_pd(_mm_dp_pd(v_, v_, 0x31)))[0];
#else
        return length2();
#endif
    }

    inline double length2() const
    {
#if __SSE2__
        return (_mm_dp_pd(v_, v_, 0x31))[0];
#else
        return v_[0] * v_[0] +v_[1] *v_[1];
#endif
    }

    inline double angle() const
    {
        return atan2(v_[1], v_[0]);
    }

    inline double & x()
    {
#if __SSE2__
        return v_[0];
#else
        return v_[0];
#endif
    }

    inline double & y()
    {
#if __SSE2__
        return v_[1];
#else
        return v_[1];
#endif
    }

    inline double const & x() const
    {
#if __SSE2__
        return v_[0];
#else
        return v_[0];
#endif
    }

    inline double const & y() const
    {
#if __SSE2__
        return v_[1];
#else
        return v_[1];
#endif
    }

    inline Vector2D & operator += (const Vector2D &other)
    {
#if __SSE2__
        v_ = _mm_add_pd(v_, other.v_);
#else
        v_[0] += other.v_[0];
        v_[1] += other.v_[1];
#endif
        return *this;
    }

    inline Vector2D & operator -= (const Vector2D &other)
    {
#if __SSE2__
        v_ = _mm_sub_pd(v_, other.v_);
#else
        v_[0] -= other.v_[0];
        v_[1] -= other.v_[1];
#endif
        return *this;
    }

    inline Vector2D & operator *= (const double d)
    {
#if __SSE2__
        v_ = _mm_mul_pd(v_, _mm_set1_pd(d));
#else
        v_[0] *= d;
        v_[1] *= d;
#endif
        return *this;
    }

    inline Vector2D & operator /= (const double d)
    {
#if __SSE2__
        v_ = _mm_div_pd(v_, _mm_set1_pd(d));
#else
        v_[0] /= d;
        v_[1] /= d;
#endif
        return *this;
    }

    inline Vector2D& operator = (const Vector2D &other)
    {
#if __SSE2__
        v_ = other.v_;
#else
        v_[0] = other.v_[0];
        v_[1] = other.v_[1];
#endif
        return *this;
    }

    inline Vector2D& operator = (Vector2D &&other)
    {
#if __SSE2__
        v_ = other.v_;
#else
        v_[0] = other.v_[0];
        v_[1] = other.v_[1];
#endif
        return *this;
    }

    inline Vector2D normalized() const
    {
#if __SSE2__
        return Vector2D(_mm_div_pd(v_, _mm_sqrt_pd(_mm_dp_pd(v_, v_, 0x31))));
#else
        return Vector2D(v_[0] / len,v_[1] / len);
#endif
    }

    inline Vector2D operator -() const
    {
#if __SSE2__
        return Vector2D(_mm_mul_pd(v_, _mm_set1_pd(-1.0)));
#else
        return Vector2D(-v_[0], -v_[1]);
#endif
    }

    inline Eigen::Vector2d toEigen() const
    {
#if __SSE2__
        return Eigen::Vector2d(v_[0], v_[1]);
#else
        return Eigen::Vector2d(v_[0],v_[1]);
#endif
    }

    inline Vector2D min(const Vector2D &other) const
    {
#if __SSE2__
        return Vector2D(_mm_min_pd(v_, other.v_));
#else
        return Vector2D(min(v_[0], other.v_[0]),
                        min(v_[1], other.v_[1]));
#endif
    }

    inline Vector2D max(const Vector2D &other) const
    {
#if __SSE2__
        return Vector2D(_mm_max_pd(v_, other.v_));
#else
        return Vector2D(max(v_[0], other.v_[0]),
                        max(v_[1], other.v_[1]));
#endif
    }

    inline double distance(const Vector2D &other) const
    {
#if __SSE2__
        __m128d d = _mm_sub_pd(v_, other.v_);
        return (_mm_sqrt_pd(_mm_dp_pd(d, d, 0x31)))[0];
#else
        return  hypot(v_[0] - other.v_[0],v_[1] - other.v_[1]);
#endif
    }

private:
    static inline double hypot (const double x, const double y)
    {
        return sqrt(x*x + y*y);
    }

#ifdef __SSE2__
    __m128d v_;
#else
    std::array<double,2> v_;
#endif

} __attribute__ ((aligned (16)));
}
}

inline std::ostream & operator << (std::ostream &out, const muse_mcl_2d::math::Vector2D &v)
{
    out << "[" << v.x() << "," << v.y() << "]";
    return out;
}

#endif // VECTOR_2D_HPP
