#ifndef KLD2D_H
#define KLD2D_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class KLD2D : public Resampling
{
public:
    KLD2D() = default;


    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;

    double kld_error_;
    double kld_z_;


};
}

#endif // KLD2D_H
