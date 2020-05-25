#ifndef MUSE_SMC_PREDICTION_HPP
#define MUSE_SMC_PREDICTION_HPP

namespace muse_smc {
template <typename PredictionModel_T, typename Data_T, typename StateSpace_T,
          typename StateIterator_T, typename Time_T>
class Prediction {
 public:
  using Ptr = std::shared_ptr<Prediction>;
  using ConstPtr = std::shared_ptr<Prediction const>;

      struct Less {
    inline bool operator()(const Prediction &lhs, const Prediction &rhs) const {
      return lhs.getStamp() < rhs.getStamp();
    }
    inline bool operator()(const Prediction::Ptr &lhs,
                           const Prediction::Ptr &rhs) const {
      return lhs->getStamp() < rhs->getStamp();
    }
  };

  struct Greater {
    inline bool operator()(const Prediction &lhs, const Prediction &rhs) const {
      const auto &lhs_stamp = lhs.getStamp();
      const auto &rhs_stamp = rhs.getStamp();
      return lhs_stamp == rhs_stamp ? lhs.stampReceived() > rhs.stampReceived()
                                    : lhs_stamp > rhs_stamp;
    }
    inline bool operator()(const Prediction::Ptr &lhs,
                           const Prediction::Ptr &rhs) const {
      const auto &lhs_stamp = lhs->getStamp();
      const auto &rhs_stamp = rhs->getStamp();
      return lhs_stamp == rhs_stamp
                 ? lhs->stampReceived() > rhs->stampReceived()
                 : lhs_stamp > rhs_stamp;
    }
  };

  inline explicit Prediction(const typename Data_T::ConstPtr &data,
                             const typename PredictionModel_T::Ptr &model)
      : data_{data}, model_{model} {}

  inline explicit Prediction(
      const typename Data_T::ConstPtr &data,
      const typename StateSpace_T::ConstPtr &state_space,
      const typename PredictionModel_T::Ptr &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Prediction() = default;

  inline auto operator()(const Time_T &until,
                         StateIterator_T states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline auto apply(const Time_T &until,
                    StateIterator_T states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline auto const &getStamp() const { return data_->timeFrame().end; }

  inline auto const &timeFrame() const { return data_->timeFrame(); }

  inline auto const &stampReceived() const { return data_->stampReceived(); }

  inline auto const &getModelName() const { return model_->getName(); }

  inline auto getModel() const { return model_; }

 private:
  typename Data_T::ConstPtr data_;
  typename StateSpace_T::ConstPtr state_space_;
  typename PredictionModel_T::Ptr model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_HPP
