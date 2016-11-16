#include "omni_drive.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::OmniDrive, muse_amcl::Propagation)

using namespace muse_amcl;

OmniDrive::OmniDrive()
{

}

void OmniDrive::apply(const Data::ConstPtr &data,
                      ParticleSet::PoseIterator set)
{

}

void OmniDrive::loadParameters(ros::NodeHandle &nh)
{

}
