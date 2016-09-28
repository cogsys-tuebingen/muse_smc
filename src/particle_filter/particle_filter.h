#pragma once

#include <vector>
#include <tf/tf.h>

#include "propagation.hpp"
#include "update.hpp"
#include "resampling.hpp"
#include "pose_generation.hpp"

namespace muse_amcl {
namespace particle_filter {
class ParticleFilter
{
public:
    ParticleFilter();

    void setPropagationFunction(const Propagation::Ptr &propagation);
    void addUpdateFunction(const Update::Ptr &update);
    void setResamplingFunction(const Resampling::Ptr &resampling);
    void setPoseGenerationFunction(const PoseGeneration::Ptr &pose_generation);

private:
    Propagation::Ptr         propagation_function_;
    Resampling::Ptr          resampling_function_;
    std::vector<Update::Ptr> update_functions_;
    PoseGeneration::Ptr      pose_generation_;

    std::vector<double>      weights_;
    std::vector<tf::Pose>    samples_;
};
}
}
