#include "multinomial.h"

#include <muse_amcl/math/random.hpp>
#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Multinomial, muse_amcl::Resampling)

using namespace muse_amcl;

void Multinomial::apply(ParticleSet &particle_set)
{
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size = p_old.size();
    ParticleSet::Particles  p_new(size);

    /// Build the cummulative sum

    std::vector<double> cumsum(size);
    cumsum[0] = p_old.front().weight_;
    for(std::size_t i = 1 ; i < size; ++i) {
        cumsum[i] = cumsum[i-1] + p_old[i].weight_;
    }

    math::random::Uniform<1> rng(cumsum.front(), 1.0);
    for(std::size_t i = 0 ; i < size ; ++i) {
        double u = rng.get();
        double q = 0;
        for(std::size_t j = 0 ; j < size ; ++j) {
            if(u > q && u <= cumsum[j]) {
                p_new[i] = p_old[j];
                break;
            }
            q = cumsum[j];
        }
    }

    std::swap(p_old, p_new);
}

void Multinomial::doSetup(ros::NodeHandle &nh_private)
{

}
