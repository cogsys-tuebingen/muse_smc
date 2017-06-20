#ifndef ANGLE_HPP
#define ANGLE_HPP

#include <cmath>
#include <complex>

namespace muse_mcl {
namespace math {
namespace angle {
const double _2_M_PI = 2.0 * M_PI;
const double _1_2_M_PI = 1.0 / _2_M_PI;
const double _1_180  = 1.0 / 180.0;
const double R_T_A = _1_180 * M_PI;       /// DEG to RAD
const double A_T_R = M_1_PI * 180.0;      /// RAD ot DEG

/**
 * @brief normalize and between -pi and pi
 * @param angle
 * @return
 */
inline double normalize(const double angle)
{
    return angle - _2_M_PI * floor((angle + M_PI) * _1_2_M_PI);
}

/**
 * @brief normalize2Pi normalizes angles within interval from [0.0, 2 * PI)
 * @param angle - angle to normalize
 * @return
 */
inline double normalize2Pi(const double angle)
{
    return angle - _2_M_PI * floor( angle * _1_2_M_PI );
}

/**
 * @brief difference calculates the normalized angle difference.
 * @param a - first angle in term
 * @param b - second angle in term
 * @return
 */
inline double difference(double a, double b)
{
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return normalize(d1);
    else
        return normalize(d2);
}

/**
 * @brief toRad converts angles given in degree to radien.
 * @param deg   - the angle to convert
 * @return      - angle in radian
 */
inline double toRad(const double deg)
{
    return deg * R_T_A;
}

/**
 * @brief fromRad converts angle from radien to degree.
 * @param rad   - the angle in radian
 * @return      - the angle in degree
 */
inline double fromRad(const double rad)
{
    return rad * A_T_R;
}

/**
 * @brief toComplex converts an angle given in radian to its complex
 *        representation.
 * @param rad   - the angle in radian
 * @return      - the angle in its complex representation
 */
inline std::complex<double> toComplex(const double rad)
{
    return std::complex<double>(std::cos(rad), std::sin(rad));
}
/**
 * @brief fromComplex converts an angle from its complex representation to
 *        radian, so it is geometrically interpretable again.
 * @param complex - the complex representation of the angle
 * @return        - the angle in radian
 */
inline double fromComplex(const std::complex<double> &complex)
{
    return std::atan2(complex.imag(), complex.real());
}
}
}
}

#endif /* ANGLE_HPP */
