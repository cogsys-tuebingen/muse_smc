#include "differential_drive.h"

#include <muse_amcl_core_plugins/odometry/odometry.hpp>
#include <muse_amcl/math/angle.hpp>
#include <muse_amcl/math/random.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::DifferentialDrive, muse_amcl::PredictionModel)

using namespace muse_amcl;

DifferentialDrive::DifferentialDrive()
{
}

PredictionModel::Result DifferentialDrive::predict(const Data::ConstPtr &data,
                                                   const ros::Time &until,
                                                   ParticleSet::Poses set)
{
    const Odometry &odometry = data->as<Odometry>();
    if(until > odometry.getTimeFrame().end) {
        return PredictionModel::Result(0, 0, data);
    } else {
        const double delta_trans     = odometry.getDelta().getOrigin().length();
        const double delta_rot1     = delta_trans >= 0.0 ? std::atan2(odometry.getDelta().getOrigin().y(),
                                                                      odometry.getDelta().getOrigin().x())
                                                         : 0.0;

        const double delta_rot2 = odometry.getDelta().getRotation().getAngle();
        const double delta_rot_noise1 = std::min(std::abs(math::angle::difference(delta_rot1, 0.0)),
                                                 std::abs(math::angle::difference(delta_rot1, M_PI)));
        const double delta_rot_noise2 = std::min(std::abs(math::angle::difference(delta_rot2, 0.0)),
                                                 std::abs(math::angle::difference(delta_rot2, M_PI)));

        auto sq = [](const double x) { return x * x; };

        math::random::Normal<1> rng_delta_rot_hat1(0.0,  std::sqrt(alpha_1_ * sq(delta_rot_noise1) +
                                                                   alpha_2_ * sq(delta_trans)),
                                                   seed_);
        math::random::Normal<1> rng_delta_trans_hat(0.0, std::sqrt(alpha_3_ * sq(delta_trans) +
                                                                   alpha_4_ * sq(delta_rot_noise1) +
                                                                   alpha_4_ * sq(delta_rot_noise2)),
                                                    seed_);
        math::random::Normal<1> rng_delta_rot_hat2(0.0,  std::sqrt(alpha_1_ * sq(delta_rot_noise2) +
                                                                   alpha_2_ * sq(delta_trans)),
                                                   seed_);
        for(math::Pose &sample : set) {
            const double delta_rot_hat1  = math::angle::difference(delta_rot1, rng_delta_rot_hat1.get());
            const double delta_trans_hat = delta_trans - rng_delta_trans_hat.get();
            const double delta_rot_hat2  = math::angle::difference(delta_rot2, rng_delta_rot_hat2.get());
            auto pose = sample.getEigen3D();
            pose(0) += delta_trans_hat * std::cos(pose(2) + delta_rot_hat1);
            pose(1) += delta_trans_hat * std::sin(pose(2) + delta_rot_hat2);
            pose(2) += delta_rot_hat1 + delta_rot_hat2;
            sample.setEigen3D(pose);
        }

        return PredictionModel::Result(delta_trans, delta_rot2);
    }
}

void DifferentialDrive::doSetup(ros::NodeHandle &nh_private)
{
    seed_    = nh_private.param<int>(privateParameter("seed"), 0);
    alpha_1_ = nh_private.param<double>(privateParameter("alpha1"), 0.1);
    alpha_2_ = nh_private.param<double>(privateParameter("alpha2"), 0.1);
    alpha_3_ = nh_private.param<double>(privateParameter("alpha3"), 0.1);
    alpha_4_ = nh_private.param<double>(privateParameter("alpha4"), 0.1);
}
