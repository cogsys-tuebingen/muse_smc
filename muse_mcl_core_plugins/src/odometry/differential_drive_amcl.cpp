#include "differential_drive_amcl.h"

#include <muse_mcl_core_plugins/odometry/odometry.hpp>
#include <muse_mcl/math/angle.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DifferentialDriveAMCL, muse_mcl::PredictionModel)

using namespace muse_mcl;

DifferentialDriveAMCL::DifferentialDriveAMCL()
{
}

DifferentialDriveAMCL::~DifferentialDriveAMCL()
{
}

PredictionModel::Result DifferentialDriveAMCL::doPredict(const Data::ConstPtr &data,
                                                     const ros::Time &until,
                                                     ParticleSet::Poses set)
{
    return PredictionModel::Result();
}

void DifferentialDriveAMCL::doSetup(ros::NodeHandle &nh_private)
{
    seed_    = nh_private.param<int>(privateParameter("seed"), 0);
    alpha_1_ = nh_private.param<double>(privateParameter("alpha1"), 0.1);
    alpha_2_ = nh_private.param<double>(privateParameter("alpha2"), 0.1);
    alpha_3_ = nh_private.param<double>(privateParameter("alpha3"), 0.1);
    alpha_4_ = nh_private.param<double>(privateParameter("alpha4"), 0.1);

    Logger &l = Logger::getLogger();
    l.info("seed_='" + std::to_string(alpha_4_) + "'",    "PredictionModel:" + name_);
    l.info("alpha_1_='" + std::to_string(alpha_1_) + "'", "PredictionModel:" + name_);
    l.info("alpha_2_='" + std::to_string(alpha_2_) + "'", "PredictionModel:" + name_);
    l.info("alpha_3_='" + std::to_string(alpha_3_) + "'", "PredictionModel:" + name_);
    l.info("alpha_4_='" + std::to_string(alpha_4_) + "'", "PredictionModel:" + name_);

}
