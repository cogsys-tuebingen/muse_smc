#ifndef MUSE_SMC_PREDICTION_HPP
#define MUSE_SMC_PREDICTION_HPP

#include <cslibs_time/time.hpp>

namespace muse_smc {
template <typename prediction_model_t, typename data_t, typename state_space_t,
          typename state_iterator_t>
class Prediction {
 public:
  using Ptr = std::shared_ptr<Prediction>;
  using ConstPtr = std::shared_ptr<Prediction const>

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

  inline explicit Prediction(const typename data_t::ConstPtr &data,
                             const typename prediction_model_t::Ptr &model)
      : data_{data}, model_{model} {}

  inline explicit Prediction(
      const typename data_t::ConstPtr &data,
      const typename state_space_t::ConstPtr &state_space,
      const typename prediction_model_t::Ptr &model)
      : data_{data}, state_space_{state_space}, model_{model} {}

  virtual ~Prediction() = default;

  inline auto operator()(const cslibs_time::Time &until,
                         typename sample_set_t::state_iterator_t states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline auto apply(const cslibs_time::Time &until,
                    typename sample_set_t::state_iterator_t states) {
    return state_space_ ? model_->apply(data_, state_space_, until, states)
                        : model_->apply(data_, until, states);
  }

  inline auto const &getStamp() const { return data_->timeFrame().end; }

  inline auto const &timeFrame() const { return data_->timeFrame(); }

  inline auto const &stampReceived() const { return data_->stampReceived(); }

  inline auto const &getModelName() const { return model_->getName(); }

  inline auto getModel() const { return model_; }

 private:
  typename data_t::ConstPtr data_;
  typename state_space_t::ConstPtr state_space_;
  typename prediction_model_t::Ptr model_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_HPP
