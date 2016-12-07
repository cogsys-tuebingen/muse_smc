#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <vector>
#include <tf/tf.h>

#include "resampling.hpp"
#include "pose_generation.hpp"

namespace muse_amcl {
namespace particle_filter {
class ParticleFilter
{
public:
    ParticleFilter();

    void setResamplingFunction(const Resampling::Ptr &resampling);
    void setPoseGenerationFunction(const PoseGeneration::Ptr &pose_generation);

private:
    Resampling::Ptr          resampling_function_;
    PoseGeneration::Ptr      pose_generation_;

    std::vector<double>      weights_;
    std::vector<tf::Pose>    samples_;
};
}
}

#endif /* PARTICLE_FILTER_H */
