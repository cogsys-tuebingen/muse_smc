#ifndef MUSE_SMC_TRAITS_SAMPLE_HPP
#define MUSE_SMC_TRAITS_SAMPLE_HPP

#include <muse_smc/samples/sample.hpp>
#include <muse_smc/smc/traits/hypothesis.hpp>
namespace muse_smc {
namespace traits {
template<typename Hypothesis_T>
struct Sample {
  using state_t = typename State<Hypothesis_T>::type;
  using state_access_t = StateAccess<Hypothesis_T>;
  using weight_t = double;
  using time_t = typename Time<Hypothesis_T>::type;
  using type = muse_smc::Sample<Hypothesis_T, state_t, state_access_t, weight_t, time_t>;
};
}
}

#endif  // MUSE_SMC_TRAITS_SAMPLE_HPP
