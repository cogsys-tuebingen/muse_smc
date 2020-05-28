#ifndef MUSE_SMC_TRAITS_STATE_SPACE_HPP
#define MUSE_SMC_TRAITS_STATE_SPACE_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/state_space/state_space.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct StateSpace {
  using state_t = typename traits::State<Hypothesis_T>::type;
  using state_space_transform_t = typename traits::Transform<Hypothesis_T>::type;
  using state_space_boundary_t =
      typename traits::StateSpaceBoundary<Hypothesis_T>::type;
  using time_t = typename traits::Time<Hypothesis_T>::type;
  using type = muse_smc::StateSpace<state_t, state_space_transform_t,
                                    state_space_boundary_t, time_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_STATE_SPACE_HPP
