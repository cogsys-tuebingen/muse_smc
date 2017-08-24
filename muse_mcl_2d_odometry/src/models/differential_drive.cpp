#include "differential_drive.h"

#include <muse_smc/math/angle.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_odometry::DifferentialDrive, muse_mcl_2d::PredictionModel2D)

using namespace muse_mcl_2d_odometry;
using namespace muse_mcl_2d;

DifferentialDrive::Result::Ptr DifferentialDrive::apply(const muse_smc::Data::ConstPtr &data,
                                                        const muse_smc::Time           &until,
                                                        sample_set_t::state_iterator_t  states)
{
    Odometry2D::ConstPtr apply;
    Odometry2D::ConstPtr leave;
    if(until < data->getTimeFrame().end) {
        Odometry2D::ConstPtr original = std::dynamic_pointer_cast<Odometry2D const>(data);
        if(!original->split(until, apply, leave)) {
            apply = original;
        }
    } else {
        apply = std::dynamic_pointer_cast<Odometry2D const>(data);
    }

    const Odometry2D &odometry = *apply;


    const double delta_trans = odometry.getDeltaLinear();
    double delta_rot1 = 0.0;
    if(delta_trans >= 0.01) {
       delta_rot1  = muse_smc::math::angle::difference(odometry.getDeltaAngularAbs(),
                                                       odometry.getStartPose().yaw());
    }
    const double delta_rot2 = muse_smc::math::angle::difference(odometry.getDeltaAngular(), delta_rot1);

    if(delta_trans < eps_zero_linear_ &&
            std::abs(delta_rot2) < eps_zero_angular_) {
        return DifferentialDrive::Result::Ptr(new Result2D(0.0, 0.0, apply, leave));
    }

    const double delta_rot_noise1 = std::min(std::abs(muse_smc::math::angle::difference(delta_rot1, 0.0)),
                                             std::abs(muse_smc::math::angle::difference(delta_rot1, M_PI)));
    const double delta_rot_noise2 = std::min(std::abs(muse_smc::math::angle::difference(delta_rot2, 0.0)),
                                             std::abs(muse_smc::math::angle::difference(delta_rot2, M_PI)));

    auto sq = [](const double x) { return x * x; };

    const double sigma_rot_hat1  = std::sqrt(alpha_1_ * sq(delta_rot_noise1) +
                                             alpha_2_ * sq(delta_trans));
    const double sigma_trans_hat = std::sqrt(alpha_3_ * sq(delta_trans) +
                                             alpha_4_ * sq(delta_rot_noise1) +
                                             alpha_4_ * sq(delta_rot_noise2));
    const double sigma_rot_hat2 =  std::sqrt(alpha_1_ * sq(delta_rot_noise2) +
                                             alpha_2_ * sq(delta_trans));

    if(!rng_delta_rot_hat1_) {
        rng_delta_rot_hat1_.reset(new muse_smc::math::random::Normal<1>(0.0,  sigma_rot_hat1, seed_));
    } else {
        rng_delta_rot_hat1_->set(0.0, sigma_rot_hat1);
    }
    if(!rng_delta_trans_hat_) {
        rng_delta_trans_hat_.reset(new muse_smc::math::random::Normal<1>(0.0, sigma_trans_hat, seed_ + 1));
    } else {
        rng_delta_trans_hat_->set(0.0, sigma_trans_hat);
    }
    if(!rng_delta_rot_hat2_) {
        rng_delta_rot_hat2_.reset(new muse_smc::math::random::Normal<1>(0.0, sigma_rot_hat2, seed_ + 2));
    } else {
        rng_delta_rot_hat2_->set(0.0, sigma_rot_hat2);
    }

    for(muse_mcl_2d::Pose2D &sample : states) {
        const double delta_rot_hat1  = muse_smc::math::angle::difference(delta_rot1, rng_delta_rot_hat1_->get());
        const double delta_trans_hat = delta_trans - rng_delta_trans_hat_->get();
        const double delta_rot_hat2  = muse_smc::math::angle::difference(delta_rot2, rng_delta_rot_hat2_->get());
        const double tx = sample.tx() + delta_trans_hat * std::cos(sample.yaw() + delta_rot_hat1);
        const double ty = sample.ty() + delta_trans_hat * std::sin(sample.yaw() + delta_rot_hat1);
        const double yaw = muse_smc::math::angle::normalize(sample.yaw() + delta_rot_hat1 + delta_rot_hat2);
        sample.setFrom(tx,ty,yaw);
    }
    return DifferentialDrive::Result::Ptr(new Result2D(delta_trans, std::abs(delta_rot2), apply, leave));
}

void DifferentialDrive::doSetup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    seed_    = nh_private.param<int>(param_name("seed"), 0);
    alpha_1_ = nh_private.param<double>(param_name("alpha1"), 0.1);
    alpha_2_ = nh_private.param<double>(param_name("alpha2"), 0.1);
    alpha_3_ = nh_private.param<double>(param_name("alpha3"), 0.1);
    alpha_4_ = nh_private.param<double>(param_name("alpha4"), 0.1);
}
