#ifndef MUSE_SMC_TRAITS_UPDATE_MODEL_HPP
#define MUSE_SMC_TRAITS_UPDATE_MODEL_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/state_space.hpp>
#include <muse_smc/update/update_model.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct UpdateModel {
  using sample_set_t = typename traits::SampleSet<Hypothesis_T>::type;
  using weight_iterator_t = typename sample_set_t::weight_iterator_t;
  using state_space_t = typename traits::StateSpace<Hypothesis_T>::type;
  using data_t = typename traits::Data<Hypothesis_T>::type;
  using time_t = typename traits::Time<Hypothesis_T>::type;
  using type = muse_smc::UpdateModel<data_t, state_space_t, weight_iterator_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_UPDATE_MODEL_HPP
