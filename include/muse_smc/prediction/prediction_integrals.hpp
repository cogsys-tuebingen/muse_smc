#ifndef PREDICTION_INTEGRALS_HPP
#define PREDICTION_INTEGRALS_HPP

#include <muse_smc/prediction/prediction_integral.hpp>
#include <unordered_map>

namespace muse_smc {
template <typename Result_T>
class PredictionIntegrals {
 public:
  using Ptr = std::shared_ptr<PredictionIntegrals<Result_T>>;
  using ConstPtr = std::shared_ptr<PredictionIntegrals<Result_T> const>;
  using prediction_integral_t = PredictionIntegral<Result_T>;

  PredictionIntegrals(
      const typename prediction_integral_t::Ptr &global_integral)
      : global_accumulator_(global_integral) {}

  virtual ~PredictionIntegrals() = default;

  inline void set(const typename prediction_integral_t::Ptr &accumulator,
                  const std::size_t id) {
    accumulators_[id] = accumulator;
  }

  inline typename prediction_integral_t::ConstPtr get(const std::size_t id) const {
    return accumulators_[id];
  }

  inline typename prediction_integral_t::ConstPtr get() const {
    return global_accumulator_;
  }

  inline void info() const { global_accumulator_->info(); }

  inline bool thresholdExceeded(const std::size_t id) const {
    auto acc = accumulators_.at(id);
    return acc->thresholdExceeded() && !acc->isZero();
  }

  inline bool thresholdExceeded() const {
    return global_accumulator_->thresholdExceeded() &&
           !global_accumulator_->isZero();
  }

  inline bool isZero(const std::size_t id) const {
    return accumulators_.at(id)->isZero();
  }

  inline bool isZero() const { return global_accumulator_->isZero(); }

  inline void reset(const std::size_t id) { accumulators_[id]->reset(); }

  inline void reset() { global_accumulator_->reset(); }

  inline void resetAll() {
    for (auto &a : accumulators_) {
      a.second->reset();
    }
  }

  inline void add(const typename Result_T::ConstPtr &step) {
    for (auto &a : accumulators_) {
      a.second->add(step);
    }
    global_accumulator_->add(step);
  }

 protected:
  typename prediction_integral_t::Ptr global_accumulator_;
  std::unordered_map<std::size_t, typename prediction_integral_t::Ptr> accumulators_;
};
}  // namespace muse_smc

#endif  // PREDICTION_INTEGRALS_HPP
