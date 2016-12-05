#include "multinomial.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Multinomial, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Multinomial::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size        = p_old.size();
    ParticleSet::Particles  p_new(size);

    /// prepare ordered sequence of random numbers
    math::random::Uniform<1> rng(0.0, 1.0);
    std::vector<double> u(size, rng.get());
    {
        auto u_it = u.rbegin();
        auto u_it_last = u_it;
        auto u_end = u.rend();
        ++u_it;
        while(u_it != u_end) {
            *u_it = rng.get() + *u_it_last;
            u_it_last = u_it;
            ++u_it;
        }
    }
    /// draw samples
    {
        auto p_new_it  = p_new.begin();
        auto p_old_it  = p_old.begin();
        double cumsum_last = 0.0;
        double cumsum = p_old_it->weight_;

        auto in_range = [cumsum, cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++p_old_it;
                cumsum_last = cumsum;
                cumsum += p_old_it->weight_;
            }

            *p_new_it = *p_old_it;
            ++p_new_it;
        }
    }
    /// swap it
    std::swap(p_old, p_new);
}

void Multinomial::doSetup(ros::NodeHandle &nh_private)
{
}
