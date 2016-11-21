#include <muse_amcl/pose_generation/pose_generation_2D.h>
#include <muse_amcl/utils/tf_eigen.hpp>

using namespace muse_amcl;
using namespace pose_generation;

PoseGeneration2D::PoseGeneration2D(ParticleSet &particle_set) :
    PoseGeneration(particle_set)
{

}

void PoseGeneration2D::normal(const tf::Pose &pose,
                              const Covariance &covariance)
{
    std::vector<Map::ConstPtr> maps;
    for(auto m : map_providers_) {
        maps.emplace_back(m->map());
    }

    ParticleSet::Particles &particles = particle_set_.getParticles();
    const std::size_t sample_size = particle_set_.maximumSampleSize();

    /// first check if mean is blocked
    bool valid = true;
    for(auto m : maps)
        valid &= m->valid(pose);

    if(!valid)
        throw std::runtime_error("Cannot initialize at given pose!");




}

void PoseGeneration2D::uniform()
{

}
