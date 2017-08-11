#ifndef TF_CONVERT_HPP
#define TF_CONVERT_HPP

#include <tf/tf.h>

#include <muse_mcl_2d/math/covariance_2d.hpp>
#include <muse_mcl_2d/math/transform_2d.hpp>

namespace muse_mcl_2d {
inline Transform2D from(const tf::Transform &t)
{
    return Transform2D(t.getOrigin().x(),
                       t.getOrigin().y(),
                       tf::getYaw(t.getRotation()));
}

inline Vector2D from(const tf::Vector3 &v)
{
    return Vector2D(v.x(), v.y());
}

}

#endif // TF_CONVERT_HPP
