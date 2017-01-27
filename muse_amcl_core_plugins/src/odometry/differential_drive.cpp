#include "differential_drive.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::DifferentialDrive, muse_amcl::PredictionModel)

using namespace muse_amcl;

DifferentialDrive::DifferentialDrive()
{
}

void DifferentialDrive::predict(const Data::ConstPtr &data,
                                ParticleSet::Poses set)
{

}

void DifferentialDrive::doSetup(ros::NodeHandle &nh)
{

}
