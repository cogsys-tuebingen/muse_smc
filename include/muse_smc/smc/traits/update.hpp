#ifndef MUSE_SMC_TRAITS_UPDATE_HPP
#define MUSE_SMC_TRAITS_UPDATE_HPP

#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/state_space.hpp>
#include <muse_smc/smc/traits/update_model.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct Update {
  using sample_set_t = typename traits::SampleSet<Sample_T>::type;
  using weight_iterator_t = typename sample_set_t::weight_iterator_t;
  using state_space_t = typename traits::StateSpace<Sample_T>::type;
  using data_t = typename traits::Data<Sample_T>::type;
  using update_model_t = typename traits::UpdateModel<Sample_T>::type;

  using type = muse_smc::Update<update_model_t, data_t, state_space_t, weight_iterator_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_UPDATE_HPP