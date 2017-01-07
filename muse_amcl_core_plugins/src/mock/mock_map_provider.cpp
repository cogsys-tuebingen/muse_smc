#include "mock_map_provider.h"

#include <class_loader/class_loader_register_macro.h>


CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockMapProvider, muse_amcl::MapProvider)

using namespace muse_amcl;

MockMapProvider::MockMapProvider()
{

}

Map::ConstPtr MockMapProvider::getMap() const
{
    return nullptr;
}

void MockMapProvider::doSetup(ros::NodeHandle &nh_private)
{

}
