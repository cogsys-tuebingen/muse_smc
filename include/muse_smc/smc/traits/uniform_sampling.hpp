#ifndef MUSE_SMC_TRAITS_UNIFORM_SAMPLING_HPP
#define MUSE_SMC_TRAITS_UNIFORM_SAMPLING_HPP

#include <muse_smc/sampling/uniform.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct UniformSampling {
  using sample_set_t = typename traits::SampleSet<Hypothesis_T>::type;
  using sample_t = typename traits::Sample<Hypothesis_T>::type;
  using type = muse_smc::UniformSampling<sample_t, sample_set_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_UNIFORM_SAMPLING_HPP
