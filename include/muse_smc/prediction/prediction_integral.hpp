#ifndef MUSE_SMC_PREDICTION_INTEGRAL_HPP
#define MUSE_SMC_PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template <typename Result_T>
class PredictionIntegral {
 public:
  using Result = Result_T;

  inline PredictionIntegral() = default;
  virtual ~PredictionIntegral() = default;

  virtual void add(const std::shared_ptr<Result const> &step) = 0;
  virtual void reset() = 0;
  virtual bool thresholdExceeded() const = 0;
  virtual bool isZero() const = 0;
  virtual void info() const = 0;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_PREDICTION_INTEGRAL_HPP
