#include "uniform_all_maps_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformAllMaps2D, muse_amcl::UniformPoseGeneration)

#include <muse_amcl/pose_generators/uniform.hpp>

using namespace muse_amcl;

using Metric   = muse_amcl::pose_generation::Metric;
using Radian   = muse_amcl::pose_generation::Radian;
using RandomPoseGenerator = muse_amcl::pose_generation::Uniform<Metric, Metric, Radian>;

UniformAllMaps2D::UniformAllMaps2D()
{
}

void UniformAllMaps2D::apply(ParticleSet &particle_set)
{
    std::cout << "test 123" << std::endl;
}

void UniformAllMaps2D::doSetup(ros::NodeHandle &nh_private)
{
    std::cout << "test 123" << std::endl;
}
