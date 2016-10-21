#include "mock_propagation.h"

#include <class_loader/class_loader_register_macro.h>

CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockPropagation, muse_amcl::Propagation)

#include <iostream>

using namespace muse_amcl;

MockPropagation::MockPropagation()
{
}

void MockPropagation::setup(const std::string &name)
{

}

void MockPropagation::apply(ParticleSet::PoseIterator set)
{
    std::cout << "Hello, I am a mock propagation - Greetings Traveller!" << std::endl;
}
