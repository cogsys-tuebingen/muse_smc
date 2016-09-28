#pragma once

#include <cmath>

namespace muse {
namespace math {
namespace angle {
const double _2_M_PI = 2.0 * M_PI;
const double _1_180  = 1.0 / 180.0;
const double _A2R = _1_180 * M_PI;
const double _R2A = M_1_PI * 180.0;


inline double normalize(const double _angle)
{
    if(fabs(_angle) < _2_M_PI)
        return _angle;

    return _angle - _2_M_PI * floor( _angle / _2_M_PI );
}

inline double difference(double _a, double _b)
{
    double d1, d2;
    _a = normalize(_a);
    _b = normalize(_b);
    d1 = _a-_b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
        d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
        return normalize(d1);
    else
        return normalize(d2);
}

inline double toRad(const double _deg)
{
    return _deg * _A2R;
}

inline double fromRad(const double _rad)
{
    return _rad * _R2A;
}
}
}
}
