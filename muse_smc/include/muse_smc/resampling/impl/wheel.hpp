#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/math/random.hpp>

namespace muse_smc {
namespace impl {
template<typename sample_t>
class WheelOfFortune
{
public:
    using sample_set_t = SampleSet<sample_t>;
    using uniform_sampling_t = UniformSampling<sample_t>;

    inline static void apply(sample_set_t &sample_set)
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

    inline static void applyRecovery(typename uniform_sampling_t::Ptr uniform_pose_sampler,
                                     const double recovery_random_pose_probability,
                                     sample_set_t &sample_set)
    {

        if(!uniform_pose_sampler->update(sample_set.getFrame())) {
            std::cerr << "[WheelOfFortune]: Updating uniform sampler didn't work, switching to normal resampling!°" << std::endl;
            apply(sample_set);
            return;
        }

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
            if(recovery_propability < recovery_random_pose_probability) {
                uniform_pose_sampler->apply(sample);
                sample.weight = recovery_propability;
            } else {
                i_p_t.insert(p_t_1[index]);
            }
        }
    }
};
}
}


#endif // WHEEL_HPP
