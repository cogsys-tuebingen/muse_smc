#ifndef TF_CONVERT_HPP
#define TF_CONVERT_HPP

#include <tf/tf.h>

#include <muse_mcl_2d/math/covariance_2d.hpp>
#include <muse_mcl_2d/math/transform_2d.hpp>

namespace muse_mcl_2d {
namespace math {
inline Vector2D from(const tf::Vector3 &v)
{
    return Vector2D(v.x(), v.y());
}

inline Transform2D from(const tf::Transform &t)
{
    return Transform2D(from(t.getOrigin()),
                       tf::getYaw(t.getRotation()));
}

inline tf::Vector3 from(const Vector2D &v)
{
    return tf::Vector3(v.x(), v.y(), 0.0);
}

inline tf::Transform from(const Transform2D &t)
{
    return tf::Transform(tf::createQuaternionFromYaw(t.yaw()),
                         from(t.translation()));
}
}
}

#endif // TF_CONVERT_HPP
