#include "stratified.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Stratified, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Stratified::apply(ParticleSet &particle_set)
{
    /// initalize particle new particle set
    ParticleSet::Particles &p_old = particle_set.getParticles();
    const std::size_t size        = p_old.size();
    ParticleSet::Particles  p_new(size);

    /// prepare ordered sequence of random numbers
    math::random::Uniform<1> rng(0.0, 1.0);
    std::vector<double> u(size);
    {
        for(std::size_t i = 0 ; i < size ; ++i) {
            u[i] = (i - 1 + rng.get()) / size;
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

void Stratified::doSetup(ros::NodeHandle &nh_private)
{

}
