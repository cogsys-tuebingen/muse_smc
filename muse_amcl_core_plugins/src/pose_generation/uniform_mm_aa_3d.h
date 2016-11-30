#ifndef MAJORMAPAA3D_H
#define MAJORMAPAA3D_H

#include <muse_amcl/particle_filter/pose_generation_uniform.hpp>

namespace muse_amcl {
class UniformMainMapAA3D : public UniformPoseGeneration
{
public:
    UniformMainMapAA3D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // MAJORMAPAA3D_H
