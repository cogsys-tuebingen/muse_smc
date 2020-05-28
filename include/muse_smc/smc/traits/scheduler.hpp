#ifndef MUSE_SMC_TRAITS_SCHEDULER_HPP
#define MUSE_SMC_TRAITS_SCHEDULER_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/update.hpp>
#include <muse_smc/smc/traits/resampling.hpp>
#include <muse_smc/scheduling/scheduler.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct Scheduler {
  using sample_set_t = typename traits::SampleSet<Hypothesis_T>::type;
  using update_t = typename traits::Update<Hypothesis_T>::type;
  using resampling_t = typename traits::Resampling<Hypothesis_T>::type;
  using type = muse_smc::Scheduler<sample_set_t, update_t, resampling_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_SCHEDULER_HPP
