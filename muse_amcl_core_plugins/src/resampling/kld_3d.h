#ifndef KLD3D_H
#define KLD3D_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class KLD3D : public Resampling
{
public:
    KLD3D() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // KLD3D_H
