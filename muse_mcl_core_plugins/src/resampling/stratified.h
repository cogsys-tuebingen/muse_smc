#ifndef STRATIFIED_H
#define STRATIFIED_H

#include <muse_mcl/plugins/types/resampling.hpp>

namespace muse_mcl {
class Stratified : public Resampling
{
public:
    Stratified() = default;

protected:
    virtual void doApply(ParticleSet &particle_set) override;
    virtual void doSetup(ros::NodeHandle &nh_private) override;
    virtual void doApplyRecovery(ParticleSet &particle_set) override;
};
}

#endif // STRATIFIED_H
