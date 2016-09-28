#include "mock_update.h"

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockUpdate, muse_amcl::Update)

using namespace muse_amcl;

MockUpdate::MockUpdate()
{

}

void MockUpdate::apply(const std::vector<tf::Pose> &poses,
                       std::vector<double> &weights)
{

}
