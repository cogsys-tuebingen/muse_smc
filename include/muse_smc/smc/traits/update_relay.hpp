#ifndef MUSE_SMC_TRAITS_UPDATE_RELAY_HPP
#define MUSE_SMC_TRAITS_UPDATE_RELAY_HPP

#include <muse_smc/smc/traits/state_space_provider.hpp>
#include <muse_smc/smc/traits/update.hpp>
#include <muse_smc/update/update_relay.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct Update {
  using update_model_t = typename traits::UpdateModel<Sample_T>::type;
  using update_t = typename traits::Update<Sample_T>::type;
  using data_provider_t = typename traits::DataProvider<Sample_T>::type;
  using data_t = typename traits::Data<Sample_T>::type;
  using state_space_provider_t =
      typename traits::StateSpaceProvider<Sample_T>::type;

  using type = muse_smc::UpdateRelay<update_model_t, update_t, data_provider_t,
                                     data_t, state_space_provider_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_UPDATE_RELAY_HPP
