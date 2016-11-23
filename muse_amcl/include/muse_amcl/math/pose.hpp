#pragma once

#include <tf/tf.h>
#include <eigen3/Eigen/Core>

namespace muse_amcl {
namespace math {
/**
 * @brief The Pose class is a helper class to provide an interface
 *        between Eigen and tf.
 */
class Pose {
public:
    using Vector6d = Eigen::Matrix<double,6,1>;
    using Vector3d = Eigen::Matrix<double,3,1>;

    /**
     * @brief Pose default contstructor.
     */
    Pose() :
        pose_(tf::createQuaternionFromRPY(0,0,0),
              tf::Vector3(0,0,0))
    {
    }

    /**
     * @brief Pose constructor for tf datatypes.
     * @param p - a pose
     */
    Pose(const tf::Pose &p) :
        pose_(p)
    {
    }

    /**
     * @brief Pose constructor for tf datatypes.
     * @param q - the rotation quaternion
     * @param p - the position
     */
    Pose(const tf::Quaternion &q,
         const tf::Point      &p) :
        pose_(q, p)
    {
    }

    /**
     * @brief Pose constructor for the 6D Eigen representation.
     * @param p
     */
    Pose(const Vector3d &p) :
        pose_(tf::createQuaternionFromRPY(0,0,p(2)),
              tf::Vector3(p(0),p(1), 0))
    {
    }

    /**
     * @brief Pose constructor for the 6D Eigen representation.
     * @param p
     */
    Pose(const Vector6d &p) :
        pose_(tf::createQuaternionFromRPY(p(3),p(4),p(5)),
              tf::Vector3(p(0),p(1),p(2)))
    {
    }

    /**
     * @brief tf returns the TF pose as reference.
     * @return
     */
    inline tf::Pose & tf()
    {
        return pose_;
    }

    /**
     * @brief tf returns the TF pose as const reference.
     * @return
     */
    inline const tf::Pose & tf() const
    {
        return pose_;
    }

    /**
     * @brief rotation returns a reference to the rotation quaternion.
     * @return
     */
    inline tf::Quaternion rotation() const
    {
        return pose_.getRotation();
    }

    /**
     * @brief rotation returns a reference to the origin.
     * @return
     */
    inline tf::Vector3 & origin()
    {
        return pose_.getOrigin();
    }

    /**
     * @brief rotation returns a const reference to the origin.
     * @return
     */
    inline const tf::Vector3 & origin() const
    {
        return pose_.getOrigin();
    }

    /**
     * @brief eigen3D returns the 2D pose as an Eigen datatype.
     * @return
     */
    inline Vector3d eigen3D() const
    {
        const tf::Vector3 &position = pose_.getOrigin();
        return Vector3d(position.x(), position.y(), tf::getYaw(pose_.getRotation()));
    }

    /**
     * @brief eigen3D returns the 3D pose as an Eigen datatype.
     * @return
     */
    inline Vector6d eigen6D() const
    {
        const tf::Vector3 &position = pose_.getOrigin();
        Vector6d e;
        e[0] = position.x();
        e[1] = position.y();
        e[2] = position.z();
        pose_.getBasis().getRPY(e[3],e[4],e[5]);
        return e;
    }

    /**
     * @brief roll returns the roll angle.
     * @return - the roll angle
     */
    inline double roll() const
    {
        double roll,pitch,yaw;
        pose_.getBasis().getRPY(roll,pitch,yaw);
        return roll;
    }

    /**
     * @brief Pitch returns the pitch angle.
     * @return - the pitch angle
     */
    inline double pitch() const
    {
        double roll,pitch,yaw;
        pose_.getBasis().getRPY(roll,pitch,yaw);
        return pitch;
    }

    /**
     * @brief yaw returns the yaw angle.
     * @return - the yaw angle
     */
    inline double yaw() const
    {
        double roll,pitch,yaw;
        pose_.getBasis().getRPY(roll,pitch,yaw);
        return yaw;
    }

    /**
     * @brief rpy returns roll, pitch and yaw angle by reference.
     * @param roll  - the roll angle
     * @param pitch - the pitch angle
     * @param yaw   - the yaw angle
     */
    inline void rpy(double &roll,
                    double &pitch,
                    double &yaw)
    {
        pose_.getBasis().getRPY(roll,pitch,yaw);
    }

private:
    tf::Pose pose_;
};
}
}
