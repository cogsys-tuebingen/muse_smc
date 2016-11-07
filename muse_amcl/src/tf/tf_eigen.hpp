#pragma once

#include <eigen3/Eigen/Core>
#include <tf/tf.h>

namespace muse_amcl {
namespace transforms {
typedef Eigen::Matrix<double, 3, 1> Pose2D;
typedef Eigen::Matrix<double, 6, 1> Pose3D;

/**
 * @brief Convert Eigen vector to tf pose.
 * @param _pose - 2D eigen pose (x,y,phi)
 * @return tf::Pose
 */
inline tf::Pose toTF(const Pose2D &_pose)
{
    return tf::Pose(tf::createQuaternionFromYaw(_pose(2)),
                    tf::Vector3(_pose(0), _pose(1), 0.0));
}

/**
 * @brief Convert Eigen vector to tf pose.
 * @param _pose - 2D eigen pose (x,y, phi)
 * @param _tf_pose - tf::Pose
 */
inline void toTF(const Pose2D &_pose,
                 tf::Pose &_tf_pose)
{
    _tf_pose.setOrigin(tf::Vector3(_pose(0), _pose(1), 0.0));
    _tf_pose.setRotation(tf::createQuaternionFromYaw(_pose(2)));
}

/**
 * @brief Convert Eigen vector to tf pose.
 *                                      yaw, pitch, roll
 * @param _pose - 2D eigen pose (x,y,z, phi, theta, psi)
 * @return tf::Pose
 */
inline tf::Pose toTF(const Pose3D &_pose)
{
    return tf::Pose(tf::createQuaternionFromRPY(_pose(5), _pose(4), _pose(3)),
                    tf::Vector3(_pose(0), _pose(1), _pose(2)));
}

/**
 * @brief Convert Eigen vector to tf pose.
 *                                      yaw, pitch, roll
 * @param _pose - 2D eigen pose (x,y,z, phi, theta, psi)
 * @param _tf_pose tf::Pose
 */
inline void toTF(const Pose3D &_pose,
                 tf::Poise &_tf_pose)
{
    _tf_pose.setOrigin(tf::Vector3(_pose(0), _pose(1), _pose(2)));
    _tf_pose.setRotation(tf::createQuaternionFromRPY(_pose(5), _pose(4), _pose(3)));
}
}
}
