#ifndef MAJORMAPAA2D_H
#define MAJORMAPAA2D_H

#include <muse_amcl/particle_filter/pose_generation_uniform.hpp>

namespace muse_amcl {
class UniformMainMap2D : public UniformPoseGeneration
{
public:
    UniformMainMap2D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:
    std::string     main_map_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif // MAJORMAPAA2D_H
