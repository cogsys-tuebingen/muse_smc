#include "wof.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::WheelOfFortune, muse_amcl::Resampling)


#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void WheelOfFortune::apply(ParticleSet &particle_set)
{
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size        = p_old.size();
    const double      w_max       = particle_set.getMaximumWeight();
    ParticleSet::Particles  p_new(size);

    math::random::Uniform<1> rng(0.0, 1.0);
    double beta = 0.0;
    std::size_t index = (std::size_t(rng.get() * size)) % size;
    for(std::size_t i = 0 ; i < size ; ++i) {
        beta += 2 * w_max * rng.get();
        while (beta > p_old[index].weight_) {
            beta -= p_old[index].weight_;
            index = (index + 1) % size;
        }
        p_new[i] = p_old[index];
    }
}

void WheelOfFortune::doSetup(ros::NodeHandle &nh_private)
{
}
