#include "multinomial.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Multinomial, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Multinomial::apply(ParticleSet &particle_set)
{
    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    ParticleSet::Insertion i_p_t = particle_set.getInsertion();

    /// prepare ordered sequence of random numbers
    const std::size_t size = p_t_1.size();
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
        auto p_t_1_it  = p_t_1.begin();
        double cumsum_last = 0.0;
        double cumsum = p_t_1_it->weight_;

        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++p_t_1_it;
                cumsum_last = cumsum;
                cumsum += p_t_1_it->weight_;
            }
            i_p_t.insert(*p_t_1_it);
        }
    }
}

void Multinomial::doSetup(ros::NodeHandle &nh_private)
{
}
