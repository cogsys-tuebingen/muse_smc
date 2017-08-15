#ifndef MULTINOMIAL_HPP
#define MULTINOMIAL_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/math/random.hpp>
#include <muse_smc/sampling/uniform.hpp>

namespace muse_smc {
namespace impl {
template<typename sample_t>
class Multinomial
{
public:
    using sample_set_t = SampleSet<sample_t>;
    using uniform_sampling_t = UniformSampling<sample_t>;

    inline static void apply(sample_set_t &sample_set)
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
    inline static  void applyRecovery(typename uniform_sampling_t::Ptr uniform_pose_sampler,
                                      const double recovery_random_pose_probability,
                                      sample_set_t &sample_set)
    {
        if(!uniform_pose_sampler->update(sample_set.getFrame())) {
            std::cerr << "[Multinomial]: Updating uniform sampler didn't work, switching to normal resampling!°" << std::endl;
            apply(sample_set);
            return;
        }

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
                if(recovery_probability < recovery_random_pose_probability) {
                    uniform_pose_sampler->apply(particle);
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
}


#endif // MULTINOMIAL_HPP
