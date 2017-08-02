#ifndef MULTINOMIAL_HPP
#define MULTINOMIAL_HPP

#include "resampling.hpp"

namespace muse {
template<typename sample_t>
class Multinomial : public Resampling<sample_t>
{
public:
    Multinomial() = default;

protected:
    virtual void doApply(sample_set_t &particle_set) override
    {
        const sample_set_t::sample_vector_t &p_t_1 = particle_set.getSamples();
        const std::size_t size = p_t_1.size();
        if(size == 0) {
            return;
        }

        sample_set_t::sample_insertion_t i_p_t = particle_set.getInsertion();

        /// prepare ordered sequence of random numbers
        math::random::Uniform<1> rng(0.0, 1.0);
        std::vector<double> u(size, std::pow(rng.get(), 1.0 / static_cast<double>(size)));
        {
            for(std::size_t k = size - 1 ; k > 0 ; --k) {
               const double u_ = std::pow(rng.get(), 1.0 / static_cast<double>(k));
               u[k-1] = u[k] * u_;
            }
        }
        /// draw samples
        {
            auto p_t_1_it  = p_t_1.begin();
            double cumsum_last = 0.0;
            double cumsum = p_t_1_it->weight_;

            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            for(auto &u_r : u) {
                while(!in_range(u_r)) {
                    ++p_t_1_it;
                    cumsum_last = cumsum;
                    cumsum += p_t_1_it->weight_;
                }
                i_p_t.insert(*p_t_1_it);
            }
        }
    }
    virtual void doApplyRecovery(ParticleSet &particle_set) override
    {
        const sample_set_t::sample_vector_t &p_t_1 = particle_set.getSamples();
        sample_set_t::sample_insertion_t i_p_t = particle_set.getInsertion();

        /// prepare ordered sequence of random numbers
        const std::size_t size = p_t_1.size();
        math::random::Uniform<1> rng(0.0, 1.0);
        std::vector<double> u(size, std::pow(rng.get(), 1.0 / static_cast<double>(size)));
        {
            for(std::size_t k = size - 1 ; k > 0 ; --k) {
               const double u_ = std::pow(rng.get(), 1.0 / static_cast<double>(k));
               u[k-1] = u[k] * u_;
            }
        }
        /// draw samples
        {
            math::random::Uniform<1> rng_recovery(0.0, 1.0);
            auto p_t_1_it  = p_t_1.begin();
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
                const double recovery_probability = rng_recovery.get();
                if(recovery_probability < recovery_random_pose_probability_) {
                    uniform_pose_sampler_->apply(particle);
                    particle.weight_ = recovery_probability;
                } else {
                    particle.pose_ = p_t_1_it->pose_;
                }
                i_p_t.insert(particle);
            }
        }
    }
};
}


#endif // MULTINOMIAL_HPP
