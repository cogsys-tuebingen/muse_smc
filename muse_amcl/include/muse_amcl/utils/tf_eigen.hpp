#pragma once

#include <eigen3/Eigen/Core>
#include <tf/tf.h>

namespace muse_amcl {
namespace conversion {
typedef Eigen::Matrix<double, 3, 1> Pose2D;
typedef Eigen::Matrix<double, 3, 3> Covariance2D;
typedef Eigen::Matrix<double, 6, 1> Pose3D;
typedef Eigen::Matrix<double, 6, 6> Covariance3D;

/**
 * @brief Convert Eigen vector to tf pose.
 * @param pose - 2D eigen pose (x,y,phi)
 * @return tf::Pose
 */
inline tf::Pose toTF(const Pose2D &pose)
{
    return tf::Pose(tf::createQuaternionFromYaw(pose(2)),
                    tf::Vector3(pose(0), pose(1), 0.0));
}

/**
 * @brief Convert Eigen vector to tf pose.
 * @param pose - 2D eigen pose (x,y, phi)
 * @param tfpose - tf::Pose
 */
inline void toTF(const Pose2D &pose,
                 tf::Pose &tfpose)
{
    tfpose.setOrigin(tf::Vector3(pose(0), pose(1), 0.0));
    tfpose.setRotation(tf::createQuaternionFromYaw(pose(2)));
}

/**
 * @brief Convert Eigen vector to tf pose.
 *                                     roll, pitch, yaw
 * @param pose - 2D eigen pose (x,y,z, phi, theta, psi)
 * @return tf::Pose
 */
inline tf::Pose toTF(const Pose3D &pose)
{
    return tf::Pose(tf::createQuaternionFromRPY(pose(3), pose(4), pose(5)),
                    tf::Vector3(pose(0), pose(1), pose(2)));
}

/**
 * @brief Convert Eigen vector to tf pose.
 *                                     roll, pitch, yaw
 * @param pose - 2D eigen pose (x,y,z, phi, theta, psi)
 * @param tfpose tf::Pose
 */
inline void toTF(const Pose3D &pose,
                 tf::Pose &tfpose)
{
    tfpose.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    tfpose.setRotation(tf::createQuaternionFromRPY(pose(3), pose(4), pose(5)));
}

/**
 * @brief Convert a tf::Pose to Pose2D.
 * @param pose - the tf pose
 * @return  Pose2D
 */
inline Pose2D toEigen2D(const tf::Pose &tfpose)
{
    return Pose2D(tfpose.getOrigin().x(),
                  tfpose.getOrigin().y(),
                  tf::getYaw(tfpose.getRotation()));
}

/**
 * @brief Convert a tf::Pose to Pose2D.
 * @param tfpose - the tf pose
 * @param pose - Pose2D
 */
inline void toEigen2D(const tf::Pose &tfpose,
                      Pose2D &pose)
{
    pose(0) = tfpose.getOrigin().x();
    pose(1) = tfpose.getOrigin().y();
    pose(2) = tf::getYaw(tfpose.getRotation());
}

/**
 * @brief Convert tf::Pose to Pose3D.
 * @param tfpose  - the tf pose
 * @return  Pose3D
 */
inline Pose3D toEigen3D(const tf::Pose &tfpose)
{
    const tf::Vector3   &origin = tfpose.getOrigin();
    const tf::Matrix3x3  rotation(tfpose.getRotation());
    Pose3D  pose;
    pose(0) = origin.x();
    pose(1) = origin.y();
    pose(2) = origin.z();
    rotation.getRPY(pose(3), pose(4), pose(5));
    return pose;
}

/**
 * @brief Convert tf::Pose to Pose3D.
 * @param tfpose - the tf pose
 * @param pose - Pose3D
 */
inline void toEigen3D(const tf::Pose &tfpose,
                      Pose3D &pose)
{
    const tf::Vector3   &origin = tfpose.getOrigin();
    const tf::Matrix3x3 rotation(tfpose.getRotation());
    pose(0) = origin.x();
    pose(1) = origin.y();
    pose(2) = origin.z();
    rotation.getRPY(pose(3), pose(4), pose(5));
}
}
}
