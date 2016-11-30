#ifndef STRATIFIED_H
#define STRATIFIED_H

#include <muse_amcl/particle_filter/resampling.hpp>

namespace muse_amcl {
class Stratified : public Resampling
{
public:
    Stratified() = default;

    virtual void apply(ParticleSet &particle_set) override;

protected:

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // STRATIFIED_H
