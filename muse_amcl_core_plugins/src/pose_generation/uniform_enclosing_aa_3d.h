#ifndef ENCLOSINGAA3D_H
#define ENCLOSINGAA3D_H

#include <muse_amcl/particle_filter/pose_generation_uniform.hpp>

namespace muse_amcl {
class UniformEnclosingAA3D : public UniformPoseGeneration
{
public:
    UniformEnclosingAA3D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(const std::map<std::string, MapProvider::Ptr>  &map_providers,
                         ros::NodeHandle &nh_private) override;

};
}

#endif // ENCLOSINGAA3D_H
