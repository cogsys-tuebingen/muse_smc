#ifndef MUSE_SMC_TRAITS_RESAMPLING_HPP
#define MUSE_SMC_TRAITS_RESAMPLING_HPP

#include <muse_smc/smc/traits/normal_sampling.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/uniform_sampling.hpp>
#include <muse_smc/resampling/resampling.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct Resampling {
  using sample_set_t = typename traits::SampleSet<Hypothesis_T>::type;
  using uniform_sampling_t = typename traits::UniformSampling<Hypothesis_T>::type;
  using normal_sampling_t = typename traits::NormalSampling<Hypothesis_T>::type;
  using weight_t = typename traits::Weight<Hypothesis_T>::type;
  using type = muse_smc::Resampling<sample_set_t, uniform_sampling_t, normal_sampling_t, weight_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_RESAMPLING_HPP
