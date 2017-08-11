#ifndef WHEEL_HPP
#define WHEEL_HPP

#include "resampling.hpp"

namespace muse_smc {
template<typename sample_t>
class WheelOfFortune : public Resampling<sample_t>
{
public:
    WheelOfFortune() = default;

protected:
    using sample_set_t = typename Resampling<sample_t>::sample_set_t;

    virtual void doApply(sample_set_t &sample_set) override
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        if(size == 0) {
            return;
        }

        const double w_max = sample_set.getMaximumWeight();
        typename sample_set_t::sample_insertion_t  i_p_t = sample_set.getInsertion();

        math::random::Uniform<1> rng(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;
        for(std::size_t i = 0 ; i < size ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight) {
                beta -= p_t_1[index].weight;
                index = (index + 1) % size;
            }
            i_p_t.insert(p_t_1[index]);
        }
    }

    virtual void doApplyRecovery(sample_set_t &sample_set) override
    {
        Resampling<sample_t>::uniform_pose_sampler_->update(sample_set.getFrame());

        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const double w_max = sample_set.getMaximumWeight();
        typename sample_set_t::sample_insertion_t  i_p_t = sample_set.getInsertion();
        const std::size_t size = p_t_1.size();

        math::random::Uniform<1> rng(0.0, 1.0);
        math::random::Uniform<1> rng_recovery(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;
        sample_t sample;
        for(std::size_t i = 0 ; i < size ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > p_t_1[index].weight) {
                beta -= p_t_1[index].weight;
                index = (index + 1) % size;
            }

            const double recovery_propability = rng_recovery.get();
            if(recovery_propability < Resampling<sample_t>::recovery_random_pose_probability_) {
                Resampling<sample_t>::uniform_pose_sampler_->apply(sample);
                sample.weight = recovery_propability;
            } else {
                i_p_t.insert(p_t_1[index]);
            }
        }
    }
};
}


#endif // WHEEL_HPP
