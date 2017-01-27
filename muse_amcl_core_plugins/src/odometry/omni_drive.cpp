#include "omni_drive.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::OmniDrive, muse_amcl::PredictionModel)

using namespace muse_amcl;

OmniDrive::OmniDrive()
{

}

void OmniDrive::predict(const Data::ConstPtr &data,
                        ParticleSet::Poses set)
{

}

void OmniDrive::doSetup(ros::NodeHandle &nh)
{

}
