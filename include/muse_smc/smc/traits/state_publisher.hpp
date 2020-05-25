#ifndef MUSE_SMC_TRAITS_STATE_PUBLISHER_HPP
#define MUSE_SMC_TRAITS_STATE_PUBLISHER_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct StatePublisher {
  using sample_set_t = typename traits::SampleSet<Sample_T>::type;
  using type = muse_smc::StatePublisher<sample_set_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_STATE_PUBLISHER_HPP
