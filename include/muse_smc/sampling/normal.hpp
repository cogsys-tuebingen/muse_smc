#ifndef MUSE_SMC_NORMAL_SAMPLING_HPP
#define MUSE_SMC_NORMAL_SAMPLING_HPP

#include <memory>

namespace muse_smc {
template <typename SampleSet_T, typename State_T, typename Covariance_T>
class NormalSampling {
 public:
  using Ptr = std::shared_ptr<NormalSampling>;
  using ConstPtr = std::shared_ptr<NormalSampling const>;

  inline NormalSampling() = default;
  virtual ~NormalSampling() = default;

  virtual bool apply(const State_T &state, const Covariance_T &covariance,
                     SampleSet_T &sample_set) = 0;
  virtual bool update(const std::string &frame) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_NORMAL_SAMPLING_HPP
