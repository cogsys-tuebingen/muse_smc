#ifndef NORMAL3D_H
#define NORMAL3D_H

#include <muse_amcl/particle_filter/pose_generation_normal.hpp>

namespace muse_amcl {
class Normal3D : public NormalPoseGeneration
{
public:
    Normal3D() = default;

    virtual void apply(const math::Pose       &pose,
                       const math::Covariance &covariance,
                       ParticleSet            &particle_set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // NORMAL3D_H
