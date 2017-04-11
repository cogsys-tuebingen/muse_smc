#include "omni_drive.h"

#include <muse_amcl_core_plugins/odometry/odometry.hpp>
#include <muse_amcl/math/angle.hpp>
#include <muse_amcl/math/random.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::OmniDrive, muse_amcl::PredictionModel)

using namespace muse_amcl;

OmniDrive::OmniDrive()
{

}

PredictionModel::Result OmniDrive::doPredict(const Data::ConstPtr &data,
                                             const ros::Time      &until,
                                             ParticleSet::Poses    set)
{
    const Odometry &odometry = data->as<Odometry>();
    auto sq = [](const double x) { return x * x; };

    const double delta_trans = odometry.getDeltaLinear();
    const double delta_rot   = odometry.getDeltaAngular();


    if(delta_trans < eps_zero_linear_ &&
            delta_rot < eps_zero_angular_)
        return PredictionModel::Result(0.0, 0.0, odometry.getTimeFrame().end - odometry.getTimeFrame().begin);



    const double delta_angle = std::atan2(odometry.getDelta().getOrigin().y(),
                                          odometry.getDelta().getOrigin().x());
    const double delta_trans_hat_stddev = std::sqrt(alpha_3_ * sq(delta_trans) +
                                                    alpha_1_ * sq(delta_rot));
    const double delta_rot_hat_stddev = std::sqrt(alpha_4_ * sq(delta_rot) +
                                                  alpha_2_ * sq(delta_trans));
    const double delta_strafe_hat_stddev = std::sqrt(alpha_1_ * sq(delta_rot) +
                                                     alpha_5_ * sq(delta_trans));


    math::random::Normal<1> rng_delta_trans_hat(0.0, delta_trans_hat_stddev, seed_);
    math::random::Normal<1> rng_delta_rot_hat(0.0, delta_rot_hat_stddev, seed_);
    math::random::Normal<1> rng_delta_strafe_hat(0.0, delta_strafe_hat_stddev, seed_);

    for(math::Pose &sample : set) {
        auto pose = sample.getEigen3D();
        const double delta_bearing = math::angle::difference(delta_angle, odometry.getStartPose().getRotation().getAngle())
                + pose(2);
        const double cos_delta_bearing = std::cos(delta_bearing);
        const double sin_delta_bearing = std::sin(delta_bearing);
        const double delta_trans_hat = delta_trans + rng_delta_trans_hat.get();
        const double delta_rot_hat = delta_rot + rng_delta_rot_hat.get();
        const double delta_strafe_hat = 0.0 + rng_delta_strafe_hat.get();
        pose(0) += (delta_trans_hat * cos_delta_bearing + delta_strafe_hat * sin_delta_bearing);
        pose(1) += (delta_trans_hat * sin_delta_bearing - delta_strafe_hat * cos_delta_bearing);
        pose(2) += delta_rot_hat;
        sample.setEigen3D(pose);
    }

    return PredictionModel::Result(delta_trans, delta_rot, odometry.getTimeFrame().end - odometry.getTimeFrame().begin);
}

void OmniDrive::doSetup(ros::NodeHandle &nh_private)
{
    seed_    = nh_private.param<int>(privateParameter("seed"), 0);
    alpha_1_ = nh_private.param<double>(privateParameter("alpha1"), alpha_1_);
    alpha_2_ = nh_private.param<double>(privateParameter("alpha1"), alpha_2_);
    alpha_3_ = nh_private.param<double>(privateParameter("alpha1"), alpha_3_);
    alpha_4_ = nh_private.param<double>(privateParameter("alpha1"), alpha_4_);
    alpha_5_ = nh_private.param<double>(privateParameter("alpha5"), 0.1);

    Logger &l = Logger::getLogger();
    l.info("seed_='" + std::to_string(alpha_4_) + "'",    "PredictionModel:" + name_);
    l.info("alpha_1_='" + std::to_string(alpha_1_) + "'", "PredictionModel:" + name_);
    l.info("alpha_2_='" + std::to_string(alpha_2_) + "'", "PredictionModel:" + name_);
    l.info("alpha_3_='" + std::to_string(alpha_3_) + "'", "PredictionModel:" + name_);
    l.info("alpha_4_='" + std::to_string(alpha_4_) + "'", "PredictionModel:" + name_);
    l.info("alpha_5_='" + std::to_string(alpha_5_) + "'", "PredictionModel:" + name_);
}
