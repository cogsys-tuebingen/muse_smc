#include "residual.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Residual, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Residual::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p = particle_set.getParticles();
    const std::size_t size    = p.size();
    ParticleSet::Particles  p_new(size);

    /// draw copies of each particle
    auto p_new_it  = p_new.begin();
    auto p_new_end = p_new.end();
    std::vector<double> u(size);
    std::vector<double> w_residual(size);
    double              n_w_residual = 0.0;
    {
        math::random::Uniform<1> rng(0.0, 1.0);
        double u_static = rng.get();
        for(std::size_t i = 0 ; i < size ; ++i) {
            const auto &sample = p[i];
            u[i] = (i + u_static) / size;
            std::size_t copies = std::floor(sample.weight_ * size);

            w_residual[i] = size * sample.weight_ - copies;
            n_w_residual += w_residual[i];

            for(std::size_t i = 0 ;
                i < copies && p_new_it != p_new_end ;
                ++i ,++p_new_it) {
                *p_new_it = sample;
            }
        }
    }
    {
        auto u_it = u.begin();
        auto p_it = p.begin();
        auto w_it = w_residual.begin();

        double cumsum_last = 0.0;
        double cumsum = 0.0;
        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        std::size_t left = std::distance(p_new_it, p_new_end);
        for(std::size_t i = 0 ; i < left ; ++i) {
            while(!in_range(*u_it)) {
                ++p_it;
                ++w_it;
                cumsum_last = cumsum;
                cumsum += *w_it / n_w_residual;
            }

            *p_new_it = *p_it;
            ++p_new_it;
            ++u_it;
        }
    }

    /// assign new content
    assert(p_new.size() == p.size());
    p.assign(p_new.begin(), p_new.end());
}

void Residual::doSetup(ros::NodeHandle &nh_private)
{

}
