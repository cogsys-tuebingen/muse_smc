#ifndef EIGEN_HPP
#define EIGEN_HPP

#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>

namespace muse_mcl {



//// convert from tf to eigen
inline void fromEigen(tf::Vector3 &tf,
                      Eigen::Vector2d &e)
{
    e(0) = tf[0];
    e(1) = tf[1];
    e(2) = 0.0;
}

inline void fromEigen(tf::Vector3 &tf,
                      Eigen::Vector3d &e)
{
    e(0) = tf[0];
    e(1) = tf[1];
    e(2) = tf[2];
}

inline void toEigen(const tf::Matrix3x3 &tf,
                    Eigen::Matrix3d &e)
{
    e(0,0)= tf[0][0];
    e(1,0)= tf[1][0];
    e(2,0)= tf[2][0];
    e(0,1)= tf[0][1];
    e(1,1)= tf[1][1];
    e(2,1)= tf[2][1];
    e(0,2)= tf[0][2];
    e(1,2)= tf[1][2];
    e(2,2)= tf[2][2];
}

inline void toEigen(const tf::Transform &tf,
                    Eigen::Affine2d &e)
{

}

inline void toEigen(const tf::Transform &tf,
                    Eigen::Affine3d &e)
{
    auto &emat = e.matrix();
    toEigen(tf.getBasis(), emat.block<3,3>(0,0));
    toEigen(tf.getOrigin(), emat.black<3,1>(0,3));
}

//// convert from eigen to tf
inline void fromEigen(const Eigen::Vector2d &e,
                      const tf::Vector3 &tf)
{
    tf[0] = e(0);
    tf[1] = e(1);
    tf[2] = 0.0;
}

inline void fromEigen(const Eigen::Vector3d &e,
                      const tf::Vector3 &tf)
{
    tf[0] = e(0);
    tf[1] = e(1);
    tf[2] = e(2);
}

inline void fromEigen(const Eigen::Matrix3d &e,
                      tf::Matrix3x3 &tf)
{
    tf[0][0] = e(0,0);
    tf[1][0] = e(1,0);
    tf[2][0] = e(2,0);
    tf[0][1] = e(0,1);
    tf[1][1] = e(1,1);
    tf[2][1] = e(2,1);
    tf[0][2] = e(0,2);
    tf[1][2] = e(1,2);
    tf[2][2] = e(2,2);
}


inline void fromEigen(const Eigen::Affine2d &e,
                      tf::Transform &tf)
{

}

inline void fromEigen(const Eigen::Affine3d &e,
                      tf::Transform &tf)
{

}









inline void getRotation(const double yaw,
                        Eigen::Matrix2d &matrix)
{
    matrix(0,0) =  cos(yaw);
    matrix(1,0) =  sin(yaw);
    matrix(0,1) = -matrix(1,0);
    matrix(1,1) =  matrix(0,0);
}

inline Eigen::Matrix2d getRotation(const double yaw)
{
    Eigen::Matrix2d matrix;
    getRotation(yaw, matrix);
    return matrix;
}

void getRotation(const double roll,
                 const double pitch,
                 const double yaw,
                 Eigen::Matrix3d &matrix)
{

    double cos_phi, sin_phi;
    sincos(yaw, &sin_phi, &cos_phi);
    double cos_theta, sin_theta;
    sincos(pitch, &sin_theta, &cos_theta);
    double cos_psi, sin_psi;
    sincos(roll, &sin_psi, &cos_psi);

    matrix(0,0) =  cos_psi * cos_phi - cos_theta * sin_phi * sin_psi;
    matrix(1,0) = -sin_psi * cos_theta - cos_theta * sin_phi * cos_psi;
    matrix(2,0) =  sin_theta * sin_phi;

    matrix(0,1) =  cos_psi * sin_phi + cos_theta * cos_phi * sin_psi;
    matrix(1,1) = -sin_psi * sin_phi + cos_theta * cos_phi * cos_psi;
    matrix(2,1) = -sin_theta * cos_phi;

    matrix(0,2) =  sin_psi * sin_theta;
    matrix(1,2) =  cos_psi * sin_theta;
    matrix(2,2) =  cos_theta;
}


inline Eigen::Matrix3d getRotation(const double roll,
                                   const double pitch,
                                   const double yaw)
{
    Eigen::Matrix3d matrix;
    getRotation(roll, pitch, yaw, matrix);
    return matrix;
}
}

#endif /* EIGEN_HPP */
