#ifndef MUSE_SMC_TRAITS_PREDICTION_INTEGRAL_HPP
#define MUSE_SMC_TRAITS_PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_integrals.hpp>
#include <muse_smc/smc/traits/prediction_model.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct PredictionIntegrals {
  using prediction_model_t = typename traits::PredictionModel<Hypothesis_T>::type;
  using prediction_result_t = typename prediction_model_t::Result;
  using type = muse_smc::PredictionIntegrals<prediction_result_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_PREDICTION_INTEGRAL_HPP
