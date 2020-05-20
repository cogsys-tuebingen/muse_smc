#ifndef PREDICTION_RELAY_HPP
#define PREDICTION_RELAY_HPP

#include <muse_smc/prediction/prediction.hpp>
#include <muse_smc/smc/smc.hpp>

namespace muse_smc {
template <typename smc_t, typename prediction_model_t, typename data_provider_t, typename state_space_provider_t>
class PredictionRelay {
 public:
  using Ptr = std::shared_ptr<PredictionRelay>;
  using prediction_t = Prediction<prediction_model_t>;
  using data_t = typename prediction_model_t::data_t;

  inline explicit PredictionRelay(const typename smc_t::Ptr &smc) : smc_{smc} {}

  virtual ~PredictionRelay() = default;

  inline void relay(const typename prediction_model_t::Ptr &p,
                    const typename data_provider_t::Ptr &d) {
    auto callback = [this, p](const typename data_t::ConstPtr &data) {
      typename prediction_t::Ptr prediction(new prediction_t(data, p));
      smc_->addPrediction(prediction);
    };
    handle_ = d->connect(callback);
  }

  inline void relay(const typename prediction_model_t::Ptr &p,
                    const typename data_provider_t::Ptr &d,
                    const typename state_space_provider_t::Ptr &s) {
    auto callback = [this, p, s](const typename data_t::ConstPtr &data) {
      const auto ss = s->getStateSpace();
      if (ss) {
        typename prediction_t::Ptr prediction(new prediction_t(data, ss, p));
        smc_->addPrediction(prediction);
      } else {
        std::cerr << "[PredictionRelay]: " << s->getName()
                  << " supplied state space which was zero!"
                  << "\n";
        std::cerr << "[PredictionRelay]: Dropped prediction!"
                  << "\n";
      }
    };
    handle_ = d->connect(callback);
  }

 private:
  typename smc_t::Ptr smc_;
  typename data_provider_t::connection_t::Ptr handle_;
};
}  // namespace muse_smc

#endif  // PREDICTION_RELAY_HPP
