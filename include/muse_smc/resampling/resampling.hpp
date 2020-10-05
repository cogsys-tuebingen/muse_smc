#ifndef MUSE_SMC_RESAMPLING_HPP
#define MUSE_SMC_RESAMPLING_HPP

#include <iostream>
#include <memory>

namespace muse_smc {
template <typename SampleSet_T, typename UniformSampling_T,
          typename NormalSampling_T, typename Weight_T>
class Resampling {
 public:
  inline Resampling() = default;
  virtual ~Resampling() = default;

  virtual void setup(
      const std::shared_ptr<UniformSampling_T> &uniform_pose_sampler,
      const std::shared_ptr<NormalSampling_T> &normal_pose_sampler,
      const Weight_T recovery_alpha_fast = Weight_T{0.0} ,
      const Weight_T recovery_alpha_slow = Weight_T{0.0} ,
      const Weight_T variance_threshold = Weight_T{0.0}) {
    uniform_pose_sampler_ = uniform_pose_sampler;
    normal_pose_sampler_ = normal_pose_sampler;
    recovery_alpha_fast_ = recovery_alpha_fast;
    recovery_alpha_slow_ = recovery_alpha_slow;
    variance_treshold_ = variance_threshold;
  }

  inline void apply(SampleSet_T &sample_set) {
    if (sample_set.getWeightSum() == Weight_T{0.0}) {
      std::cerr << "[MuseSMC]: All particle weights are zero. \n";
      return;
    }

    if (sample_set.getWeightVariance() < variance_treshold_) {
      std::cerr << "[MuseSMC]: Variance not high enough for resampling. \n";
      return;
    }

    auto do_apply = [&sample_set, this]() { doApply(sample_set); };
    auto do_apply_recovery = [&sample_set, this]() {
      doApplyRecovery(sample_set);
      resetRecovery();
    };

    recovery_random_pose_probability_ == Weight_T{0.0} ? do_apply() : do_apply_recovery();
  }

  inline void resetRecovery() {
    recovery_fast_ = Weight_T{0.0} ;
    recovery_slow_ = Weight_T{0.0} ;
  }

  inline void updateRecovery(SampleSet_T &particle_set) {
    const Weight_T weight_average = particle_set.getAverageWeight();
    if (recovery_slow_ == Weight_T{0.0}) {
      recovery_slow_ = weight_average;
    } else {
      recovery_slow_ +=
          recovery_alpha_slow_ * (weight_average - recovery_slow_);
    }

    if (recovery_fast_ == Weight_T{0.0}) {
      recovery_fast_ = weight_average;
    } else {
      recovery_fast_ +=
          recovery_alpha_fast_ * (weight_average - recovery_fast_);
    }

    if (recovery_slow_ != Weight_T{0.0}) {
      recovery_random_pose_probability_ =
          std::max(Weight_T{0.0} , Weight_T{1.0}  - recovery_fast_ / recovery_slow_);
    }
  }

 protected:
  Weight_T recovery_alpha_fast_{0.0};
  Weight_T recovery_alpha_slow_{0.0};
  Weight_T recovery_fast_{0.0};
  Weight_T recovery_slow_{0.0};
  Weight_T recovery_random_pose_probability_{0.0};
  Weight_T variance_treshold_{0.0};
  std::shared_ptr<UniformSampling_T> uniform_pose_sampler_;
  std::shared_ptr<NormalSampling_T> normal_pose_sampler_;

  virtual void doApply(SampleSet_T &sample_set) = 0;
  virtual void doApplyRecovery(SampleSet_T &sample_set) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_RESAMPLING_HPP
