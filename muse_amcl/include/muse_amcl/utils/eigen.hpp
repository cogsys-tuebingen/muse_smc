#pragma once

#include <eigen3/Eigen/Geometry>

namespace muse_amcl {

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
