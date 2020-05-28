#ifndef MUSE_SMC_TRAITS_REQUESTS_HPP
#define MUSE_SMC_TRAITS_REQUESTS_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>

#include <muse_smc/smc/request_state_initilization.hpp>
#include <muse_smc/smc/request_uniform_initilization.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct RequestStateInitialization {
  using time_t = typename traits::Time<Hypothesis_T>::type;
  using state_t = typename traits::State<Hypothesis_T>::type;
  using covariance_t = typename traits::Covariance<Hypothesis_T>::type;
  using type = muse_smc::RequestStateInitialization<time_t, state_t, covariance_t>;
};

template <typename Hypothesis_T>
struct RequestUniformInitialization {
  using time_t = typename traits::Time<Hypothesis_T>::type;
  using type = muse_smc::RequestUniformInitialization<time_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_REQUESTS_HPP
