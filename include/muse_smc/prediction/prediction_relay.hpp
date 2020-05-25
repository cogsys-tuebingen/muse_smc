#ifndef MUSE_SMC_PREDICTION_RELAY_HPP
#define MUSE_SMC_PREDICTION_RELAY_HPP

namespace muse_smc {
template <typename SMC_T, typename prediction_model_t, typename prediction_t,
          typename data_provider_t, typename data_t,
          typename state_space_provider_t>
class PredictionRelay {
 public:
  using Ptr = std::shared_ptr<PredictionRelay>;
  using ConstPtr = std::shared_ptr<PredictionRelay const>;

  inline explicit PredictionRelay(const typename SMC_T::Ptr &smc) : smc_{smc} {}

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
  typename SMC_T::Ptr smc_;
  typename data_provider_t::connection_t::Ptr handle_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_RELAY_HPP
