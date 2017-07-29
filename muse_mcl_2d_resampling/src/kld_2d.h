#ifndef KLD_2D_H
#define KLD_2D_H

#include <muse_mcl/resampling/resampling.hpp>

namespace muse_mcl {
class KLD2D : public Resampling
{
public:
    KLD2D() = default;


protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;

    double kld_error_;              /// error between histgram based distribution and particle filter posterior
    double kld_z_;                  /// upper standard normal quantile for (1-p) distributions
    double recovery_alpha_slow_;    ///
    double recovery_alpha_fast_;    ///
};
}

#endif // KLD_2D_H
