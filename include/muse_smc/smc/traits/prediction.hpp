#ifndef MUSE_SMC_TRAITS_PREDICTION_HPP
#define MUSE_SMC_TRAITS_PREDICTION_HPP

#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/smc/traits/prediction_model.hpp>
#include <muse_smc/smc/traits/sample.hpp>
#include <muse_smc/smc/traits/sample_set.hpp>
#include <muse_smc/smc/traits/state_space.hpp>

namespace muse_smc {
namespace traits {
template <typename Hypothesis_T>
struct Prediction {
  using sample_set_t = typename traits::SampleSet<Hypothesis_T>::type;
  using state_iterator_t = typename sample_set_t::state_iterator_t;
  using state_space_t = typename traits::StateSpace<Hypothesis_T>::type;
  using data_t = typename traits::Data<Hypothesis_T>::type;
  using prediction_model_t = typename traits::PredictionModel<Hypothesis_T>::type;
  using time_t = typename traits::Time<Hypothesis_T>::type;
  using time_frame_t = typename traits::TimeFrame<Hypothesis_T>::type;
  using type = muse_smc::Prediction<prediction_model_t, data_t, state_space_t,
                                    state_iterator_t, time_t, time_frame_t>;
};
}  // namespace traits
}  // namespace muse_smc
#endif  // MUSE_SMC_TRAITS_PREDICTION_HPP
