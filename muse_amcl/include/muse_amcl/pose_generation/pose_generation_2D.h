#pragma once

#include "../particle_filter/pose_generation.hpp"

namespace muse_amcl {
namespace pose_generation {
class PoseGeneration2D : public PoseGeneration
{
public:
    void normal(const math::Pose       &pose,
                const math::Covariance &covariance,
                ParticleSet            &particle_set) override;

    void uniform();

protected:
    void doSetup(ros::NodeHandle &nh_private);


};
}
}
