#include <muse_amcl/pose_generation/pose_generation_3D.h>

using namespace muse_amcl;
using namespace pose_generation;

PoseGeneration3D::PoseGeneration3D(ParticleSet &particle_set) :
    PoseGeneration(particle_set)
{

}

void PoseGeneration3D::normal(const math::Pose &pose,
                              const math::Covariance &covariance)
{

}

void PoseGeneration3D::uniform()
{

}

void PoseGeneration3D::doSetup(ros::NodeHandle &nh_private)
{

}
