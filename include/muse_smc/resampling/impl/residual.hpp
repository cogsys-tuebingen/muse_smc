#ifndef RESIDUAL_HPP
#define RESIDUAL_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <cslibs_math/random/random.hpp>

#include <iostream>

namespace muse_smc {
namespace impl {
template<typename sample_t>
class Residual
{
public:
    using sample_set_t        = SampleSet<sample_t>;
    using uniform_sampling_t  = UniformSampling<sample_t>;

    inline static void apply(sample_set_t &sample_set)
    {
        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        const std::size_t size = p_t_1.size();
        assert(size != 0);

        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        std::vector<double> u(size);
        std::vector<double> w_residual(size);
        double              n_w_residual = 0.0;
        std::size_t         i_p_t_size = 0;
        {
            cslibs_math::random::Uniform<double,1> rng(0.0, 1.0);
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


    inline static  void applyRecovery(typename uniform_sampling_t::Ptr uniform_pose_sampler,
                                      const double recovery_random_pose_probability,
                                      sample_set_t &sample_set)
    {
        if(!uniform_pose_sampler->update(sample_set.getFrame())) {
            std::cerr << "[Residual]: Updating uniform sampler didn't work, switching to normal resampling!°" << "\n";
            apply(sample_set);
            return;
        }

        const typename sample_set_t::sample_vector_t &p_t_1 = sample_set.getSamples();
        typename sample_set_t::sample_insertion_t i_p_t = sample_set.getInsertion();

        cslibs_math::random::Uniform<double,1> rng_recovery(0.0, 1.0);

        const std::size_t size = p_t_1.size();
        std::vector<double> u(size);
        std::vector<double> w_residual(size);
        double              n_w_residual = 0.0;
        std::size_t         i_p_t_size = 0;
        {
            cslibs_math::random::Uniform<double,1> rng(0.0, 1.0);
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
                    if(recovery_probability < recovery_random_pose_probability) {
                        uniform_pose_sampler->apply(sample);
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
                if(recovery_probability < recovery_random_pose_probability) {
                    uniform_pose_sampler->apply(sample);
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
}

#endif // RESIDUAL_HPP
