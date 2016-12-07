#include "normal_2d.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Normal2D, muse_amcl::NormalPoseGeneration)

#include <muse_amcl/pose_generators/normal.hpp>

using namespace muse_amcl;

using M   = muse_amcl::pose_generation::Metric;
using R   = muse_amcl::pose_generation::Radian;
using RNG = muse_amcl::pose_generation::Normal<M, M, R>;

void Normal2D::apply(const math::Pose       &pose,
                     const math::Covariance &covariance,
                     ParticleSet            &particle_set)
{

}

void Normal2D::doSetup(ros::NodeHandle &nh_private)
{

}
