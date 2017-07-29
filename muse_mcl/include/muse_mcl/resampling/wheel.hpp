#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "resampling.hpp"

namespace muse_mcl {
class WheelOfFortune : public Resampling
{
public:
    WheelOfFortune() = default;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override
    {

    }

    virtual void doApply(ParticleSet &particle_set) override
    {
        const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
        const std::size_t size = p_t_1.size();
        if(size == 0) {
            return;
        }

        const double w_max = particle_set.getSampleWeightMaximum();
        Insertion  i_p_t = particle_set.getInsertion();

        math::random::Uniform<1> rng(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;
        for(std::size_t i = 0 ; i < size ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight_) {
                beta -= p_t_1[index].weight_;
                index = (index + 1) % size;
            }
            i_p_t.insert(p_t_1[index]);
        }
    }

    virtual void doApplyRecovery(ParticleSet &particle_set) override
    {
        uniform_pose_sampler_->update(particle_set.getFrame());

        const ParticleSet::Particles &p_t_1 = particle_set.getSamples();
        const double w_max = particle_set.getSampleWeightMaximum();
        Insertion  i_p_t = particle_set.getInsertion();
        const std::size_t size = p_t_1.size();

        math::random::Uniform<1> rng(0.0, 1.0);
        math::random::Uniform<1> rng_recovery(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;
        Particle particle;
        for(std::size_t i = 0 ; i < size ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight_) {
                beta -= p_t_1[index].weight_;
                index = (index + 1) % size;
            }

            const double recovery_propability = rng_recovery.get();
            if(recovery_propability < recovery_random_pose_probability_) {
                uniform_pose_sampler_->apply(particle);
                particle.weight_ = recovery_propability;
            } else {
                i_p_t.insert(p_t_1[index]);
            }
        }
    }
};
}


#endif // WHEEL_HPP
