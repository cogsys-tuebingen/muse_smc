#ifndef MUSE_SMC_SYSTEMATIC_HPP
#define MUSE_SMC_SYSTEMATIC_HPP

#include <cslibs_math/random/random.hpp>
#include <iostream>

namespace muse_smc {
namespace impl {
template <typename SampleSet_T, typename UniformSampling_T>
class Systematic {
 public:
  inline static void apply(SampleSet_T &sample_set) {
    const auto &p_t_1 = sample_set.getSamples();
    const auto size = p_t_1.size();
    assert(size != 0);
    auto i_p_t = sample_set.getInsertion();

    /// prepare ordered sequence of random numbers
    std::vector<double> u(size);
    {
      cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
      double u_static = rng.get();

      for (std::size_t i = 0; i < size; ++i) {
        u[i] = (i + u_static) / size;
      }
    }
    /// draw samples
    {
      auto p_t_1_it = p_t_1.begin();
      double cumsum_last = 0.0;
      double cumsum = p_t_1_it->weight;

      auto in_range = [&cumsum, &cumsum_last](double u) {
        return u >= cumsum_last && u < cumsum;
      };

      for (auto &u_r : u) {
        while (!in_range(u_r)) {
          ++p_t_1_it;
          cumsum_last = cumsum;
          cumsum += p_t_1_it->weight;
        }
        i_p_t.insert(*p_t_1_it);
      }
    }
  }

  inline static void applyRecovery(
      typename UniformSampling_T::Ptr uniform_pose_sampler,
      const double recovery_random_pose_probability, SampleSet_T &sample_set) {
    if (!uniform_pose_sampler->update(sample_set.getFrame())) {
      std::cerr << "[Systematic]: Updating uniform sampler didn't work, "
                   "switching to normal resampling!°"
                << "\n";
      apply(sample_set);
      return;
    }

    const auto &p_t_1 = sample_set.getSamples();
    auto i_p_t = sample_set.getInsertion();

    /// prepare ordered sequence of random numbers
    const auto size = p_t_1.size();
    std::vector<double> u(size);
    {
      cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
      double u_static = rng.get();

      for (std::size_t i = 0; i < size; ++i) {
        u[i] = (i + u_static) / size;
      }
    }
    /// draw samples
    {
      cslibs_math::random::Uniform<double, 1> rng_recovery(0.0, 1.0);
      auto p_t_1_it = p_t_1.begin();
      double cumsum_last = 0.0;
      double cumsum = p_t_1_it->weight;

      auto in_range = [&cumsum, &cumsum_last](double u) {
        return u >= cumsum_last && u < cumsum;
      };

      typename SampleSet_T::sample_t sample;

      for (auto &u_r : u) {
        while (!in_range(u_r)) {
          ++p_t_1_it;
          cumsum_last = cumsum;
          cumsum += p_t_1_it->weight;
        }
        const double recovery_probability = rng_recovery.get();
        if (recovery_probability < recovery_random_pose_probability) {
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
}  // namespace impl
}  // namespace muse_smc

#endif  // MUSE_SMC_SYSTEMATIC_HPP
