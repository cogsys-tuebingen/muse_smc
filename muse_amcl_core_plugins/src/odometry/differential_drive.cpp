#include "differential_drive.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::DifferentialDrive, muse_amcl::Propagation)

using namespace muse_amcl;

DifferentialDrive::DifferentialDrive()
{
}


void DifferentialDrive::setup(const std::string &name)
{

}

void DifferentialDrive::apply(ParticleSet::PoseIterator set)
{

}
