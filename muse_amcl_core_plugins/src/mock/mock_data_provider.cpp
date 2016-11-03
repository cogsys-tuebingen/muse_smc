#include "mock_data_provider.h"

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockDataProvider, muse_amcl::DataProvider)

using namespace muse_amcl;

MockDataProvider::MockDataProvider()
{

}

void MockDataProvider::loadParameters(ros::NodeHandle &nh_private)
{

}
