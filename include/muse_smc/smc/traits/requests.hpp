#ifndef MUSE_SMC_TRAITS_REQUESTS_HPP
#define MUSE_SMC_TRAITS_REQUESTS_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct RequestStateInitialization {
  using time_t = cslibs_time::Time;
  using state_t = traits::State<Sample_T>::type;
  using covariance_t = traits::Covariance<Sample_t>::type;
  using type = RequestStateInitialization<time_t, state_t, covariance_t>;
};

template <typename Sample_T>
struct RequestUniformInitialization {
  using time_t = cslibs_time::Time;
  using type = RequestUniformInitialization<time_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_REQUESTS_HPP
