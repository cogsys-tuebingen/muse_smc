#ifndef MUSE_SMC_TRAITS_PREDICTION_RELAY_HPP
#define MUSE_SMC_TRAITS_PREDICTION_RELAY_HPP

#include <muse_smc/prediction/prediction_relay.hpp>
#include <muse_smc/smc/smc.hpp>
#include <muse_smc/smc/traits/prediction.hpp>
#include <muse_smc/smc/traits/state_space_provider.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct PredictionRelay {
  using prediction_model_t = typename traits::PredictionModel<Hypothesis_T>::type;
  using prediction_t = typename traits::Prediction<Hypothesis_T>::type;
  using data_provider_t = typename traits::DataProvider<Hypothesis_T>::type;
  using data_t = typename traits::Data<Hypothesis_T>::type;
  using state_space_provider_t =
      typename traits::StateSpaceProvider<Hypothesis_T>::type;
  using smc_t = muse_smc::SMC<Hypothesis_T>;

  using type = muse_smc::PredictionRelay<smc_t, prediction_model_t,
                                         prediction_t, data_provider_t, data_t,
                                         state_space_provider_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_PREDICTION_RELAY_HPP
