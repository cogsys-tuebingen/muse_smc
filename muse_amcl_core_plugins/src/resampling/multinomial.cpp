#include "multinomial.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Multinomial, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Multinomial::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p = particle_set.getParticles();
    const std::size_t size        = p.size();
    ParticleSet::Particles  p_new(size);

    /// prepare ordered sequence of random numbers
    std::size_t k = size;
    math::random::Uniform<1> rng(0.0, 1.0);
    std::vector<double> u(size, std::pow(rng.get(), 1.0 / k));
    {
        auto u_it = u.rbegin();
        auto u_it_last = u_it;
        auto u_end = u.rend();
        ++u_it;
        while(u_it != u_end) {
            *u_it = *u_it_last * std::pow(rng.get(), 1.0 / k);
            u_it_last = u_it;
            ++u_it;
            --k;
        }
    }
    /// draw samples
    {
        auto p_new_it  = p_new.begin();
        auto p_it  = p.begin();
        double cumsum_last = 0.0;
        double cumsum = p_it->weight_;

        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++p_it;
                cumsum_last = cumsum;
                cumsum += p_it->weight_;
            }

            *p_new_it = *p_it;
            ++p_new_it;
        }
    }

    /// assign new content
    assert(p_new.size() == p.size());
    std::swap(p_new, p);
}

void Multinomial::doSetup(ros::NodeHandle &nh_private)
{
}
