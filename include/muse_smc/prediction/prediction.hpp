#ifndef MUSE_SMC_PREDICTION_HPP
#define MUSE_SMC_PREDICTION_HPP

#include <string>

namespace muse_smc {
template <typename PredictionModel_T, typename Data_T, typename StateSpace_T,
          typename StateIterator_T, typename Time_T, typename TimeFrame_T>
class Prediction {
 public:
  struct Less {
    inline bool operator()(const Prediction &lhs, const Prediction &rhs) const {
      return lhs.getStamp() < rhs.getStamp();
    }
    inline bool operator()(const std::shared_ptr<Prediction> &lhs,
                           const std::shared_ptr<Prediction> &rhs) const {
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
    inline bool operator()(const std::shared_ptr<Prediction> &lhs,
                           const std::shared_ptr<Prediction> &rhs) const {
      const auto &lhs_stamp = lhs->getStamp();
      const auto &rhs_stamp = rhs->getStamp();
      return lhs_stamp == rhs_stamp
                 ? lhs->stampReceived() > rhs->stampReceived()
                 : lhs_stamp > rhs_stamp;
    }
  };

  inline explicit Prediction(const std::shared_ptr<Data_T const> &data,
                             const std::shared_ptr<PredictionModel_T> &model)
      : data_{data}, model_{model} {}

  inline explicit Prediction(const std::shared_ptr<Data_T const> &data,
                             const std::shared_ptr<StateSpace_T const> &state_space,
                             const std::shared_ptr<PredictionModel_T> &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Prediction() = default;

  inline std::shared_ptr<typename PredictionModel_T::Result> operator()(const Time_T &until, StateIterator_T states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline std::shared_ptr<typename PredictionModel_T::Result> apply(const Time_T &until, StateIterator_T states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline Time_T const &getStamp() const { return data_->timeFrame().end; }

  inline TimeFrame_T const &timeFrame() const { return data_->timeFrame(); }

  inline Time_T const &stampReceived() const { return data_->stampReceived(); }

  inline std::string const &getModelName() const { return model_->getName(); }

  inline std::shared_ptr<PredictionModel_T> getModel() const { return model_; }

 private:
  std::shared_ptr<Data_T const> data_;
  std::shared_ptr<StateSpace_T const> state_space_;
  std::shared_ptr<PredictionModel_T> model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_HPP
