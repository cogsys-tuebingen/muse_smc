#include "normal_3d.h"

#include <muse_amcl/pose_generators/normal.hpp>

using namespace muse_amcl;

using M   = muse_amcl::pose_generation::Metric;
using R   = muse_amcl::pose_generation::Radian;
using RNG = muse_amcl::pose_generation::Normal<M, M, M, R, R, R>;

void Normal3D::apply(const math::Pose       &pose,
                     const math::Covariance &covariance,
                     ParticleSet            &particle_set)
{

}

void Normal3D::doSetup(const std::map<std::string, MapProvider::Ptr> &map_providers,
                       ros::NodeHandle &nh_private)
{

}
