#ifndef SYSTEMATIC_HPP
#define SYSTEMATIC_HPP

#include "resampling.hpp"

namespace muse_smc {
template<typename sample_t>
class Systematic : public Resampling<sample_t>
{
public:
    Systematic() = default;

protected:
    using sample_set_t = typename Resampling<sample_t>::sample_set_t;

    virtual void doApply(sample_set_t &sample_set) override
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        if(size == 0) {
            return;
        }

        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

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
            auto p_t_1_it = p_t_1.begin();
            double cumsum_last = 0.0;
            double cumsum = p_t_1_it->weight;

            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            for(auto &u_r : u) {
                while(!in_range(u_r)) {
                    ++p_t_1_it;
                    cumsum_last = cumsum;
                    cumsum += p_t_1_it->weight;
                }
                i_p_t.insert(*p_t_1_it);
            }
        }
    }
    virtual void doApplyRecovery(sample_set_t &sample_set) override
    {
        Resampling<sample_t>::uniform_pose_sampler_->update(sample_set.getFrame());

        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

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
            double cumsum = p_t_1_it->weight;

            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            sample_t sample;
            for(auto &u_r : u) {
                while(!in_range(u_r)) {
                    ++p_t_1_it;
                    cumsum_last = cumsum;
                    cumsum += p_t_1_it->weight;
                }
                const double recovery_probability = rng_recovery.get();
                if(recovery_probability < Resampling<sample_t>::recovery_random_pose_probability_) {
                    Resampling<sample_t>::uniform_pose_sampler_->apply(sample);
                    sample.weight = recovery_probability;
                    i_p_t.insert(sample);
                } else {
                    i_p_t.insert(*p_t_1_it);
                }
            }
        }
    }
};
}

#endif // SYSTEMATIC_HPP
