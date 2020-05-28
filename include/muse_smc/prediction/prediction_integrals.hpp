#ifndef MUSE_SMC_PREDICTION_INTEGRALS_HPP
#define MUSE_SMC_PREDICTION_INTEGRALS_HPP

#include <muse_smc/prediction/prediction_integral.hpp>
#include <unordered_map>

namespace muse_smc {
template <typename Result_T>
class PredictionIntegrals {
 public:
  using prediction_integral_t = PredictionIntegral<Result_T>;

  PredictionIntegrals(
      const std::shared_ptr<prediction_integral_t> &global_integral)
      : global_accumulator_(global_integral) {}

  virtual ~PredictionIntegrals() = default;

  inline void set(const std::shared_ptr<prediction_integral_t> &accumulator,
                  const std::size_t id) {
    accumulators_[id] = accumulator;
  }

  inline std::shared_ptr<prediction_integral_t const> get(
      const std::size_t id) const {
    return accumulators_[id];
  }

  inline std::shared_ptr<prediction_integral_t const> get() const {
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

  inline void add(const std::shared_ptr<Result_T const> &step) {
    for (auto &a : accumulators_) {
      a.second->add(step);
    }
    global_accumulator_->add(step);
  }

 protected:
  std::shared_ptr<prediction_integral_t> global_accumulator_;
  std::unordered_map<std::size_t, std::shared_ptr<prediction_integral_t>>
      accumulators_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_INTEGRALS_HPP
