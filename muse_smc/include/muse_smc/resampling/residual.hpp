#ifndef RESIDUAL_HPP
#define RESIDUAL_HPP

#include "resampling.hpp"

namespace muse_smc {
template<typename sample_t>
class Residual : public Resampling<sample_t>
{
public:
    Residual() = default;

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

        std::vector<double> u(size);
        std::vector<double> w_residual(size);
        double              n_w_residual = 0.0;
        std::size_t         i_p_t_size = 0;
        {
            math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();
            for(std::size_t i = 0 ; i < size ; ++i) {
                const auto &sample = p_t_1[i];
                u[i] = (i + u_static) / size;
                std::size_t copies = std::floor(sample.weight * size);

                w_residual[i] = size * sample.weight - copies;
                n_w_residual += w_residual[i];

                for(std::size_t i = 0 ; i < copies && i_p_t_size < size ;
                    ++i ,++i_p_t_size) {
                    i_p_t.insert(sample);
                }
            }
        }
        {
            auto u_it = u.begin();
            auto p_t_1_it = p_t_1.begin();
            auto w_it = w_residual.begin();

            double cumsum_last = 0.0;
            double cumsum = 0.0;
            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            for(std::size_t i = i_p_t_size ; i < size ; ++i) {
                while(!in_range(*u_it)) {
                    ++p_t_1_it;
                    ++w_it;
                    cumsum_last = cumsum;
                    cumsum += *w_it / n_w_residual;
                }
                i_p_t.insert(*p_t_1_it);
                ++u_it;
            }
        }
    }

    virtual void doApplyRecovery(sample_set_t &sample_set) override
    {
        Resampling<sample_t>::uniform_pose_sampler_->update();

        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        math::random::Uniform<1> rng_recovery(0.0, 1.0);

        const std::size_t size = p_t_1.size();
        std::vector<double> u(size);
        std::vector<double> w_residual(size);
        double              n_w_residual = 0.0;
        std::size_t         i_p_t_size = 0;
        {
            math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();
            for(std::size_t i = 0 ; i < size ; ++i) {
                const auto &sample_p_t_1 = p_t_1[i];
                u[i] = (i + u_static) / size;
                std::size_t copies = std::floor(sample_p_t_1.weight * size);

                w_residual[i] = size * sample_p_t_1.weight - copies;
                n_w_residual += w_residual[i];

                sample_t sample;
                for(std::size_t i = 0 ; i < copies && i_p_t_size < size ;
                    ++i ,++i_p_t_size) {
                    const double recovery_probability = rng_recovery.get();
                    if(recovery_probability < Resampling<sample_t>::recovery_random_pose_probability_) {
                        Resampling<sample_t>::uniform_pose_sampler_->apply(sample);
                        sample.weight = recovery_probability;
                        i_p_t.insert(sample);
                    } else {
                        i_p_t.insert(sample);
                    }
                }
            }
        }
        {
            auto u_it = u.begin();
            auto p_t_1_it = p_t_1.begin();
            auto w_it = w_residual.begin();

            double cumsum_last = 0.0;
            double cumsum = 0.0;
            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            sample_t sample;
            for(std::size_t i = i_p_t_size ; i < size ; ++i) {
                while(!in_range(*u_it)) {
                    ++p_t_1_it;
                    ++w_it;
                    cumsum_last = cumsum;
                    cumsum += *w_it / n_w_residual;
                }

                const double recovery_probability = rng_recovery.get();
                if(recovery_probability < Resampling<sample_t>::recovery_random_pose_probability_) {
                    Resampling<sample_t>::uniform_pose_sampler_->apply(sample);
                    sample.weight = recovery_probability;
                    i_p_t.insert(sample);
                } else {
                    i_p_t.insert(*p_t_1_it);
                }
                ++u_it;
            }
        }
    }
};
}

#endif // RESIDUAL_HPP
