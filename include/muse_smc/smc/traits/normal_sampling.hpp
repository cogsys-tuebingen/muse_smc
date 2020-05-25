#ifndef MUSE_SMC_TRAITS_NORMAL_SAMPLING_HPP
#define MUSE_SMC_TRAITS_NORMAL_SAMPLING_HPP

#include <muse_smc/sampling/normal.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct NormalSampling {
  using sample_set_t = typename traits::SampleSet<Sample_T>::type;
  using state_t = typename traits::State<Sample_T>::type;
  using covariance_t = typename traits::Covariance<Sample_T>::type;
  using type = muse_smc::NormalSampling<sample_set_t, state_t, covariance_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_NORMAL_SAMPLING_HPP
