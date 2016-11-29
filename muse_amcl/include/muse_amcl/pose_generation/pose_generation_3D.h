#pragma once

#include "../particle_filter/pose_generation.hpp"

namespace muse_amcl {
namespace pose_generation {
class PoseGeneration3D : public PoseGeneration
{
public:
    void normal(const math::Pose &pose,
                const math::Covariance &covariance,
                ParticleSet &particle_set) override;

    void uniform(ParticleSet &particle_set);

protected:
    void doSetup(ros::NodeHandle &nh_private);

};
}
}

