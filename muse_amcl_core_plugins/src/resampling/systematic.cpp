#include "systematic.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Systematic, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

#include "impl/systematic.hpp"

using namespace muse_amcl;

void Systematic::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    const ParticleSet::Particles p_t_1 = particle_set.getSamples();
    ParticleInsertion i_p_t = particle_set.getInsertion();

    /// prepare ordered sequence of random numbers
    std::vector<double> u(size);
    {
        math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i + u_static) / size;
        }
    }
    /// draw samples
    {
        auto src_it = p_t_1.begin();
        double cumsum_last = 0.0;
        double cumsum = src_it->weight_;

        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++src_it;
                cumsum_last = cumsum;
                cumsum += src_it->weight_;
            }
            i_p_t.insert(*src_it);
        }
    }
}

void Systematic::doSetup(ros::NodeHandle &nh_private)
{

}
