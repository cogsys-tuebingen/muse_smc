#ifndef ENCLOSINGAA2D_H
#define ENCLOSINGAA2D_H

#include <muse_amcl/particle_filter/pose_generation_uniform.hpp>

namespace muse_amcl {
class UniformEnclosingAA2D : public UniformPoseGeneration
{
public:
    UniformEnclosingAA2D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // ENCLOSINGAA2D_H
