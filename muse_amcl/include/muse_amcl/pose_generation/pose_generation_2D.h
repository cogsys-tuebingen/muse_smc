#pragma once

#include "../particle_filter/pose_generation.hpp"

namespace muse_amcl {
namespace pose_generation {
class PoseGeneration2D : public PoseGeneration
{
public:
    PoseGeneration2D(ParticleSet &particle_set);

    void normal(const math::Pose &pose,
                const math::Covariance &covariance) override;
    void uniform();

};
}
}
