#ifndef MUSE_SMC_PREDICTION_RELAY_HPP
#define MUSE_SMC_PREDICTION_RELAY_HPP

namespace muse_smc {
template <typename SMC_T, typename prediction_model_t, typename prediction_t,
          typename data_provider_t, typename data_t,
          typename state_space_provider_t>
class PredictionRelay {
 public:
  inline explicit PredictionRelay(const std::shared_ptr<SMC_T> &smc)
      : smc_{smc} {}

  virtual ~PredictionRelay() = default;

  inline void relay(const std::shared_ptr<prediction_model_t> &p,
                    const std::shared_ptr<data_provider_t> &d) {
    auto callback = [this, p](const std::shared_ptr<data_t const> &data) {
      std::shared_ptr<prediction_t> prediction(new prediction_t(data, p));
      smc_->addPrediction(prediction);
    };
    handle_ = d->connect(callback);
  }

  inline void relay(const std::shared_ptr<prediction_model_t> &p,
                    const std::shared_ptr<data_provider_t> &d,
                    const std::shared_ptr<state_space_provider_t> &s) {
    auto callback = [this, p, s](const std::shared_ptr<data_t const> &data) {
      const auto ss = s->getStateSpace();
      if (ss) {
        std::shared_ptr<prediction_t> prediction(new prediction_t(data, ss, p));
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
  std::shared_ptr<SMC_T> smc_;
  std::shared_ptr<typename data_provider_t::connection_t> handle_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_RELAY_HPP
