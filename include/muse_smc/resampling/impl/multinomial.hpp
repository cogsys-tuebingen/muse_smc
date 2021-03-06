#ifndef MUSE_SMC_MULTINOMIAL_HPP
#define MUSE_SMC_MULTINOMIAL_HPP

#include <cslibs_math/random/random.hpp>
#include <cslibs_math/sampling/uniform.hpp>

namespace muse_smc {
namespace impl {
template <typename SampleSet_T, typename UniformSampling_T>
class Multinomial {
 public:
  inline static void apply(SampleSet_T &sample_set) {
    const auto &p_t_1 = sample_set.getSamples();
    const auto size = p_t_1.size();
    assert(size != 0);

    auto i_p_t = sample_set.getInsertion();

    /// prepare ordered sequence of random numbers
    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    std::vector<double> u(size,
                          std::pow(rng.get(), 1.0 / static_cast<double>(size)));
    {
      for (std::size_t k = size - 1; k > 0; --k) {
        const double u_ = std::pow(rng.get(), 1.0 / static_cast<double>(k));
        u[k - 1] = u[k] * u_;
      }
    }
    /// draw samples
    {
      auto p_t_1_it = p_t_1.begin();
      double cumsum_last = 0.0;
      double cumsum = p_t_1_it->weight();

      auto in_range = [&cumsum, &cumsum_last](double u) {
        return u >= cumsum_last && u < cumsum;
      };

      for (auto &u_r : u) {
        while (!in_range(u_r)) {
          ++p_t_1_it;
          cumsum_last = cumsum;
          cumsum += p_t_1_it->weight();
        }
        i_p_t.insert(*p_t_1_it);
      }
    }
  }

  inline static void applyRecovery(
      std::shared_ptr<UniformSampling_T> uniform_pose_sampler,
      const double recovery_random_pose_probability, SampleSet_T &sample_set) {
    if (!uniform_pose_sampler->update(sample_set.getFrame())) {
      std::cerr << "[Multinomial]: Updating uniform sampler didn't work, "
                   "switching to normal resampling!°"
                << "\n";
      apply(sample_set);
      return;
    }

    const auto &p_t_1 = sample_set.getSamples();
    auto i_p_t = sample_set.getInsertion();

    /// prepare ordered sequence of random numbers
    const std::size_t size = p_t_1.size();
    cslibs_math::random::Uniform<double, 1> rng(0.0, 1.0);
    std::vector<double> u(size,
                          std::pow(rng.get(), 1.0 / static_cast<double>(size)));
    {
      for (std::size_t k = size - 1; k > 0; --k) {
        const double u_ = std::pow(rng.get(), 1.0 / static_cast<double>(k));
        u[k - 1] = u[k] * u_;
      }
    }
    /// draw samples
    {
      cslibs_math::random::Uniform<double, 1> rng_recovery(0.0, 1.0);
      auto p_t_1_it = p_t_1.begin();
      double cumsum_last = 0.0;
      double cumsum = p_t_1_it->weight();

      auto in_range = [&cumsum, &cumsum_last](double u) {
        return u >= cumsum_last && u < cumsum;
      };

      typename SampleSet_T::sample_t sample;
      for (auto &u_r : u) {
        while (!in_range(u_r)) {
          ++p_t_1_it;
          cumsum_last = cumsum;
          cumsum += p_t_1_it->weight();
        }
        const double recovery_probability = rng_recovery.get();
        if (recovery_probability < recovery_random_pose_probability) {
          uniform_pose_sampler->apply(sample);
          sample.weight() = recovery_probability;
        } else {
          sample.state() = p_t_1_it->state();
        }
        i_p_t.insert(sample);
      }
    }
  }
};
}  // namespace impl
}  // namespace muse_smc

#endif  // MUSE_SMC_MULTINOMIAL_HPP
