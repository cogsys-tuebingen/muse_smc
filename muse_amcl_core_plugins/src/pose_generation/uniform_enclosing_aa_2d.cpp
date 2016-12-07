#include "uniform_enclosing_aa_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::UniformEnclosingAA2D, muse_amcl::UniformPoseGeneration)

#include <muse_amcl/pose_generators/uniform.hpp>

using namespace muse_amcl;

using M   = muse_amcl::pose_generation::Metric;
using R   = muse_amcl::pose_generation::Radian;
using RNG = muse_amcl::pose_generation::Uniform<M, M, R>;

void UniformEnclosingAA2D::apply(ParticleSet &particle_set)
{

}

void UniformEnclosingAA2D::doSetup(ros::NodeHandle &nh_private)
{

}
