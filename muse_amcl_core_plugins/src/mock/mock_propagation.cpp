#include "mock_propagation.h"

#include <iostream>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockPropagation, muse_amcl::PredictionModel)

using namespace muse_amcl;

MockPropagation::MockPropagation()
{
}

PredictionModel::Result MockPropagation::predict(const Data::ConstPtr &data,
                                                 const ros::Time &until,
                                                 ParticleSet::Poses set)
{
    std::cout << "Hello, I am a mock propagation - Greetings Traveller!" << std::endl;
    std::cout <<  first_parameter << " " << second_parameter << std::endl;
}

void MockPropagation::doSetup(ros::NodeHandle &nh)
{
    first_parameter  = nh.param(parameter("first"), 0.5);
    second_parameter = nh.param(parameter("second"),1);
}
