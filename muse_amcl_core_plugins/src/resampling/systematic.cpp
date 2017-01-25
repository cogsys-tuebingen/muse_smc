#include "systematic.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::Systematic, muse_amcl::Resampling)

#include <muse_amcl/math/random.hpp>

using namespace muse_amcl;

void Systematic::doSetup(ros::NodeHandle &nh_private)
{
}

void Systematic::doApply(ParticleSet &particle_set)
{
    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    ParticleSet::Insertion i_p_t = particle_set.getInsertion();

    /// prepare ordered sequence of random numbers
    const std::size_t size = p_t_1.size();
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
        auto p_t_1_it = p_t_1.begin();
        double cumsum_last = 0.0;
        double cumsum = p_t_1_it->weight_;

        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        Particle particle;
        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++p_t_1_it;
                cumsum_last = cumsum;
                cumsum += p_t_1_it->weight_;
            }
            particle.pose_ = p_t_1_it->pose_;
            i_p_t.insert(particle);
        }
    }
}

void Systematic::doApplyRecovery(ParticleSet &particle_set)
{
    uniform_pose_sampler_->update(particle_set.getFrame());

    const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
    ParticleSet::Insertion i_p_t = particle_set.getInsertion();

    /// prepare ordered sequence of random numbers
    const std::size_t size = p_t_1.size();
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
        math::random::Uniform<1> rng_recovery(0.0, 1.0);
        auto p_t_1_it = p_t_1.begin();
        double cumsum_last = 0.0;
        double cumsum = p_t_1_it->weight_;

        auto in_range = [&cumsum, &cumsum_last] (double u)
        {
            return u >= cumsum_last && u < cumsum;
        };

        Particle particle;
        for(auto &u_r : u) {
            while(!in_range(u_r)) {
                ++p_t_1_it;
                cumsum_last = cumsum;
                cumsum += p_t_1_it->weight_;
            }
            if(rng_recovery.get() < recovery_random_pose_probability_) {
                uniform_pose_sampler_->apply(particle);
            } else {
                particle.pose_ = p_t_1_it->pose_;
            }
            i_p_t.insert(particle);
        }
    }
}
