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
    std::vector<double> seq(size, rng.get());
    {
        auto seq_it = seq.rbegin();
        auto seq_it_last = seq_it;
        auto seq_end = seq.rend();
        ++seq_it;
        while(seq_it != seq_end) {
            *seq_it = rng.get() + *seq_it_last;
            seq_it_last = seq_it;
            ++seq_it;
        }
    }
    /// resampling starts here
    {
        double cumsum_last = 0.0;
        double cumsum = 0.0;
        auto in_range = [cumsum, cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };
        auto seq_it   = seq.begin();
        auto seq_end  = seq.end();
        auto p_new_it = p_new.begin();
        for(const auto &p : p_old) {
            cumsum += p.weight_;
            while(seq_it != seq_end && in_range(*seq_it)) {
                *p_new_it = p;
            }
            if(seq_it == seq_end)
                break;
        }
    }
    /// swap it
    std::swap(p_old, p_new);
}

void Multinomial::doSetup(ros::NodeHandle &nh_private)
{
}
