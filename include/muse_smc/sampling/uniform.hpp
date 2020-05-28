#ifndef MUSE_SMC_UNIFORM_SAMPLING_HPP
#define MUSE_SMC_UNIFORM_SAMPLING_HPP

#include <memory>

namespace muse_smc {
template <typename Sample_T, typename SampleSet_T>
class UniformSampling {
 public:
  virtual bool apply(SampleSet_T &sample_set) = 0;
  virtual void apply(Sample_T &sample) = 0;
  virtual bool update(const std::string &frame) = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_UNIFORM_SAMPLING_HPP
