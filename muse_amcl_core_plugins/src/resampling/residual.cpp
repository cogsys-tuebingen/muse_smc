#include "residual.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Residual, muse_amcl::Resampling)

using namespace muse_amcl;

void Residual::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size        = p_old.size();
    ParticleSet::Particles  p_new(size);

    /// draw copies of each particle
    auto p_new_it = p_new.begin();
    auto p_new_end = p_new.end();
    for(const auto &p : p_old) {
        std::size_t copies = std::floor(p._weight_ * size);
        for(std::size_t i = 0 ; i < copies && p_new_it != p_new_end ; ++i ,++p_new_it) {
            *p_new_it = p;
        }
    }

    /// if particles to draw are left, do it systematically
    std::size_t left = std::distance(p_new_it, p_new_end);
    if(left > 0) {
        /// prepare ordered sequence of random numbers
        std::vector<double> u(left);
        {
            math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();
            for(std::size_t i = 0 ; i < size ; ++i) {
                u[i] = (i - 1 + u_static) / size;
            }
        }
    }



    /// draw samples
    {
        double cumsum_last = 0.0;
        double cumsum = 0.0;
        auto in_range = [cumsum, cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };
        auto u_it   = u.begin();
        auto u_end  = u.end();
        auto p_new_it = p_new.begin();
        for(const auto &p : p_old) {
            cumsum += p.weight_;
            while(u_it != u_end && in_range(*u_it)) {
                *p_new_it = p;
            }
            if(u_it == u_end)
                break;
        }
    }
    /// swap it
    std::swap(p_old, p_new);
}

void Residual::doSetup(ros::NodeHandle &nh_private)
{

}
