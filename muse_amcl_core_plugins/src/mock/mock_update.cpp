#include "mock_update.h"

#include <class_loader/class_loader_register_macro.h>

#include <iostream>

CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockUpdate, muse_amcl::Update)

using namespace muse_amcl;

MockUpdate::MockUpdate()
{

}

double MockUpdate::apply(const Data::ConstPtr &data,
                         const Map::ConstPtr &map,
                         ParticleSet::Weights set)
{
    std::cout << "Hello, I am a mock update - Greetings Traveller!" << std::endl;
    std::cout <<  first_parameter << " " << second_parameter << std::endl;
    return 0.0;
}

void MockUpdate::doSetup(ros::NodeHandle &nh_private)
{
    first_parameter  = nh_private.param(parameter("first"), 0.5);
    second_parameter = nh_private.param(parameter("second"),1);
}
