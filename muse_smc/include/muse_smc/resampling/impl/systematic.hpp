#ifndef SYSTEMATIC_HPP
#define SYSTEMATIC_HPP

#include <iostream>

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <cslibs_math/random/random.hpp>

namespace muse_smc {
namespace impl {
template<typename state_space_description_t>
class Systematic
{
public:
    using sample_t            = typename state_space_description_t::sample_t;
    using sample_set_t        = SampleSet<state_space_description_t>;
    using uniform_sampling_t  = UniformSampling<state_space_description_t>;

    inline static void apply(sample_set_t &sample_set)
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
            cslibs_math::random::Uniform<1> rng(0.0, 1.0);
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

    inline static  void applyRecovery(typename uniform_sampling_t::Ptr uniform_pose_sampler,
                                      const double recovery_random_pose_probability,
                                      sample_set_t &sample_set)
    {
        if(!uniform_pose_sampler->update(sample_set.getFrame())) {
            std::cerr << "[Systematic]: Updating uniform sampler didn't work, switching to normal resampling!°" << "\n";
            apply(sample_set);
            return;
        }

        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        /// prepare ordered sequence of random numbers
        const std::size_t size = p_t_1.size();
        std::vector<double> u(size);
        {
            cslibs_math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();

            for(std::size_t i = 0 ; i < size ; ++i) {
                u[i] = (i + u_static) / size;
            }
        }
        /// draw samples
        {
            cslibs_math::random::Uniform<1> rng_recovery(0.0, 1.0);
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
                if(recovery_probability < recovery_random_pose_probability) {
                    uniform_pose_sampler->apply(sample);
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
}
#endif // SYSTEMATIC_HPP
