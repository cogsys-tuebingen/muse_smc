#include "differential_drive_abs.h"

#include <muse_mcl_2d_odometry/odometry/odometry.hpp>
#include <muse_mcl/math/angle.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DifferentialDriveAbs, muse_mcl::ModelPrediction)

using namespace muse_mcl;

DifferentialDriveAbs::DifferentialDriveAbs()
{
}

ModelPrediction::Result DifferentialDriveAbs::doPredict(const Data::ConstPtr &data,
                                                        const ros::Time &until,
                                                        ParticleSet::Poses set)
{
    const Odometry &odometry = data->as<Odometry>();
    const double delta_trans = odometry.getDeltaLinear();
    const double delta_rot1  = delta_trans < 0.01 ? 0.0 :
                                                    math::angle::difference(odometry.getDeltaAngularAbs(),
                                                                            odometry.getStartPose().yaw());

    const double delta_rot2 = math::angle::difference(odometry.getDeltaAngular(), delta_rot1);

    if(delta_trans < eps_zero_linear_ &&
            delta_rot2 < eps_zero_angular_)
        return ModelPrediction::Result(0.0, 0.0, data);


    const double delta_rot_noise1 = std::min(std::abs(math::angle::difference(delta_rot1, 0.0)),
                                             std::abs(math::angle::difference(delta_rot1, M_PI)));
    const double delta_rot_noise2 = std::min(std::abs(math::angle::difference(delta_rot2, 0.0)),
                                             std::abs(math::angle::difference(delta_rot2, M_PI)));

    if(!rng_delta_rot_hat1_) {
        rng_delta_rot_hat1_.reset(new math::random::Normal<1>(0.0,  std::sqrt(alpha_1_ * std::abs(delta_rot_noise1) +
                                                                              alpha_2_ * std::abs(delta_trans)),
                                                              seed_));
    } else {
        rng_delta_rot_hat1_->set(0.0,  std::sqrt(alpha_1_ * std::abs(delta_rot_noise1) +
                                                 alpha_2_ * std::abs(delta_trans)));
    }
    if(!rng_delta_trans_hat_) {
        rng_delta_trans_hat_.reset(new math::random::Normal<1>(0.0, std::sqrt(alpha_3_ * std::abs(delta_trans) +
                                                                              alpha_4_ * std::abs(delta_rot_noise1) +
                                                                              alpha_4_ * std::abs(delta_rot_noise2)),
                                                              seed_));
    } else {
        rng_delta_trans_hat_->set(0.0, std::sqrt(alpha_3_ * std::abs(delta_trans) +
                                                 alpha_4_ * std::abs(delta_rot_noise1) +
                                                 alpha_4_ * std::abs(delta_rot_noise2)));
    }
    if(!rng_delta_rot_hat2_) {
        rng_delta_rot_hat2_.reset(new math::random::Normal<1>(0.0,  std::sqrt(alpha_1_ * std::abs(delta_rot_noise2) +
                                                                              alpha_2_ * std::abs(delta_trans)),
                                                              seed_));
    } else {
        rng_delta_rot_hat2_->set(0.0,  std::sqrt(alpha_1_ * std::abs(delta_rot_noise2) +
                                                 alpha_2_ * std::abs(delta_trans)));
    }

    for(math::Pose &sample : set) {
        const double delta_rot_hat1  = math::angle::difference(delta_rot1, rng_delta_rot_hat1_->get());
        const double delta_trans_hat = delta_trans - rng_delta_trans_hat_->get();
        const double delta_rot_hat2  = math::angle::difference(delta_rot2, rng_delta_rot_hat2_->get());
        auto pose = sample.getEigen3D();
        pose(0) += delta_trans_hat * std::cos(pose(2) + delta_rot_hat1);
        pose(1) += delta_trans_hat * std::sin(pose(2) + delta_rot_hat1);
        pose(2) += delta_rot_hat1 + delta_rot_hat2;
        sample.setEigen3D(pose);
    }

    return ModelPrediction::Result(delta_trans, delta_rot2, data);
}

void DifferentialDriveAbs::doSetup(ros::NodeHandle &nh_private)
{
    seed_    = nh_private.param<int>(privateParameter("seed"), 0);
    alpha_1_ = nh_private.param<double>(privateParameter("alpha1"), 0.1);
    alpha_2_ = nh_private.param<double>(privateParameter("alpha2"), 0.1);
    alpha_3_ = nh_private.param<double>(privateParameter("alpha3"), 0.1);
    alpha_4_ = nh_private.param<double>(privateParameter("alpha4"), 0.1);
}
