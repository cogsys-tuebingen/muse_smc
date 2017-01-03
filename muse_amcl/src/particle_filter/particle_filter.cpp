#include "particle_filter.h"

using namespace muse_amcl;
using namespace particle_filter;

ParticleFilter::ParticleFilter()
{
}

void ParticleFilter::setResamplingFunction(const Resampling::Ptr &resampling)
{
    resampling_function_ = resampling;
}

void ParticleFilter::setPoseGenerationFunction(const PoseGeneration::Ptr &pose_generation)
{
    pose_generation_ = pose_generation;
}