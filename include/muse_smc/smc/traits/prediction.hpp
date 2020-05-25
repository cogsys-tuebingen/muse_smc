#ifndef MUSE_SMC_TRAITS_PREDICTION_HPP
#define MUSE_SMC_TRAITS_PREDICTION_HPP

#include <muse_smc/smc/traits/prediction_model.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/state_space.hpp>

#include <muse_smc/prediction/prediction.hpp>

namespace muse_smc {
namespace traits {
template <typename Sample_T>
struct Prediction {
  using sample_set_t = typename traits::SampleSet<Sample_T>::type;
  using state_iterator_t = typename sample_set_t::state_iterator;
  using state_space_t = typename traits::StateSpace<Sample_T>::type;
  using data_t = typename traits::Data<Sample_T>::type;
  using prediction_model_t = typename traits::PredictionModel<Sample_T>::type;
  using time_t = typename traits::Time<Sample_T>::type;
  using type = muse_smc::Prediction<prediction_model_t, data_t,
                                         state_space_t, state_iterator_t, time_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_PREDICTION_HPP
