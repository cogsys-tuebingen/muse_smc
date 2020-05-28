#ifndef MUSE_SMC_WHEEL_HPP
#define MUSE_SMC_WHEEL_HPP

#include <cslibs_math/random/random.hpp>
#include <iostream>

namespace muse_smc {
namespace impl {
template <typename SampleSet_T, typename UniformSampling_T>
class WheelOfFortune {
 public:
  static void apply(SampleSet_T &sample_set) {
    const auto &p_t_1 = sample_set.getSamples();
    const std::size_t size = p_t_1.size();
    assert(size != 0);

    const double w_max = sample_set.getMaximumWeight();
    auto i_p_t = sample_set.getInsertion();

    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    double beta = 0.0;
    std::size_t index = (std::size_t(rng.get() * size)) % size;

    for (std::size_t i = 0; i < size; ++i) {
      beta += 2 * w_max * rng.get();
      while (beta > p_t_1[index].weight()) {
        beta -= p_t_1[index].weight();
        index = (index + 1) % size;
      }
      i_p_t.insert(p_t_1[index]);
    }
  }

  static void applyRecovery(
      typename UniformSampling_T::Ptr uniform_pose_sampler,
      const double recovery_random_pose_probability, SampleSet_T &sample_set) {
    if (!uniform_pose_sampler->update(sample_set.getFrame())) {
      std::cerr << "[WheelOfFortune]: Updating uniform sampler didn't work, "
                   "switching to normal resampling!°"
                << "\n";
      apply(sample_set);
      return;
    }

    const auto &p_t_1 = sample_set.getSamples();
    const double w_max = sample_set.getMaximumWeight();
    auto i_p_t = sample_set.getInsertion();
    const auto size = p_t_1.size();

    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    cslibs_math::random::Uniform<double, 1> rng_recovery(0.0, 1.0);
    double beta = 0.0;
    std::size_t index = (std::size_t(rng.get() * size)) % size;
    typename SampleSet_T::sample_t sample;

    for (std::size_t i = 0; i < size; ++i) {
      beta += 2 * w_max * rng.get();
      while (beta > p_t_1[index].weight()) {
        beta -= p_t_1[index].weight();
        index = (index + 1) % size;
      }

      const double recovery_propability = rng_recovery.get();
      if (recovery_propability < recovery_random_pose_probability) {
        uniform_pose_sampler->apply(sample);
        sample.weight() = recovery_propability;
      } else {
        i_p_t.insert(p_t_1[index]);
      }
    }
  }
};
}  // namespace impl
}  // namespace muse_smc

#endif  // MUSE_SMC_WHEEL_HPP
