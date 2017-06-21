#include "differential_drive.h"

#include <muse_mcl_core_plugins/odometry/odometry.hpp>
#include <muse_mcl/math/angle.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DifferentialDrive, muse_mcl::PredictionModel)

using namespace muse_mcl;

DifferentialDrive::DifferentialDrive()
{
}

DifferentialDrive::~DifferentialDrive()
{
}

PredictionModel::Result DifferentialDrive::doPredict(const Data::ConstPtr &data,
                                                     const ros::Time &until,
                                                     ParticleSet::Poses set)
{

    Odometry::ConstPtr apply;
    Odometry::ConstPtr leave;
    if(until < data->getTimeFrame().end) {
        Odometry::ConstPtr original = std::dynamic_pointer_cast<Odometry const>(data);
        if(!original->split(until, apply, leave)) {
            apply = original;
        }
    } else {
        apply = std::dynamic_pointer_cast<Odometry const>(data);
    }

    const Odometry &odometry = *apply;


    const double delta_trans = odometry.getDeltaLinear();
    double delta_rot1 = 0.0;
    if(delta_trans >= 0.01) {
       delta_rot1  = math::angle::difference(odometry.getDeltaAngularAbs(),
                                             odometry.getStartPose().yaw());
    }
    const double delta_rot2 = math::angle::difference(odometry.getDeltaAngular(), delta_rot1);

    if(delta_trans < eps_zero_linear_ &&
            std::abs(delta_rot2) < eps_zero_angular_) {
        return PredictionModel::Result(0.0, 0.0, data);
    }

    const double delta_rot_noise1 = std::min(std::abs(math::angle::difference(delta_rot1, 0.0)),
                                             std::abs(math::angle::difference(delta_rot1, M_PI)));
    const double delta_rot_noise2 = std::min(std::abs(math::angle::difference(delta_rot2, 0.0)),
                                             std::abs(math::angle::difference(delta_rot2, M_PI)));

    auto sq = [](const double x) { return x * x; };

    const double sigma_rot_hat1 = std::sqrt(alpha_1_ * sq(delta_rot_noise1) +
                                            alpha_2_ * sq(delta_trans));
    const double sigma_trans_hat = std::sqrt(alpha_3_ * sq(delta_trans) +
                                             alpha_4_ * sq(delta_rot_noise1) +
                                             alpha_4_ * sq(delta_rot_noise2));
    const double sigma_rot_hat2 =  std::sqrt(alpha_1_ * sq(delta_rot_noise2) +
                                             alpha_2_ * sq(delta_trans));

    if(!rng_delta_rot_hat1_) {
        rng_delta_rot_hat1_.reset(new math::random::Normal<1>(0.0,  sigma_rot_hat1, seed_));
    } else {
        rng_delta_rot_hat1_->set(0.0, sigma_rot_hat1);
    }
    if(!rng_delta_trans_hat_) {
        rng_delta_trans_hat_.reset(new math::random::Normal<1>(0.0, sigma_trans_hat, seed_ + 1));
    } else {
        rng_delta_trans_hat_->set(0.0, sigma_trans_hat);
    }
    if(!rng_delta_rot_hat2_) {
        rng_delta_rot_hat2_.reset(new math::random::Normal<1>(0.0, sigma_rot_hat2, seed_ + 2));
    } else {
        rng_delta_rot_hat2_->set(0.0, sigma_rot_hat2);
    }

    for(math::Pose &sample : set) {
        const double delta_rot_hat1  = math::angle::difference(delta_rot1, rng_delta_rot_hat1_->get());
        const double delta_trans_hat = delta_trans - rng_delta_trans_hat_->get();
        const double delta_rot_hat2  = math::angle::difference(delta_rot2, rng_delta_rot_hat2_->get());
        auto pose = sample.getEigen3D();
        pose(0) += delta_trans_hat * std::cos(pose(2) + delta_rot_hat1);
        pose(1) += delta_trans_hat * std::sin(pose(2) + delta_rot_hat1);
        pose(2)  = math::angle::normalize(pose(2) + delta_rot_hat1 + delta_rot_hat2);
        sample.setEigen3D(pose);
    }
    return PredictionModel::Result(delta_trans, std::abs(delta_rot2), apply, leave);
}

void DifferentialDrive::doSetup(ros::NodeHandle &nh_private)
{
    seed_    = nh_private.param<int>(privateParameter("seed"), 0);
    alpha_1_ = nh_private.param<double>(privateParameter("alpha1"), 0.1);
    alpha_2_ = nh_private.param<double>(privateParameter("alpha2"), 0.1);
    alpha_3_ = nh_private.param<double>(privateParameter("alpha3"), 0.1);
    alpha_4_ = nh_private.param<double>(privateParameter("alpha4"), 0.1);
}
