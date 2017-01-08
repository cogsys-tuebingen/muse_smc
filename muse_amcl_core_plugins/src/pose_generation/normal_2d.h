#ifndef NORMAL2D_H
#define NORMAL2D_H

#include <muse_amcl/particle_filter/pose_generation_normal.hpp>

namespace muse_amcl {
class Normal2D : public NormalPoseGeneration
{
public:
    Normal2D();

    virtual void apply(const math::Pose       &pose,
                       const math::Covariance &covariance,
                       ParticleSet            &particle_set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // NORMAL2D_H
