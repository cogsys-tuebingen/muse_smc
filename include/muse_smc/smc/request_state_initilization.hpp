#ifndef MUSE_SMC_REQUEST_STATE_INITIALIZATION_HPP
#define MUSE_SMC_REQUEST_STATE_INITIALIZATION_HPP

#include <eigen3/Eigen/Core>

namespace muse_smc {
template <typename Time_T, typename State_T, typename Covariance_T>
class EIGEN_ALIGN16 RequestStateInitialization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using allocator_t = Eigen::aligned_allocator<RequestStateInitialization>;

  inline explicit RequestStateInitialization(const Time_T &time,
                                            const State_T &state,
                                            const Covariance_T &covariance)
      : time_{time}, state_{state}, covariance_{covariance} {}

  const Time_T &time() const { return time_; }
  const State_T &state() const { return state_; }
  const Covariance_T &covariance() { return covariance_; }

 private:
  const Time_T time_;
  const State_T state_;
  const Covariance_T covariance_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_REQUEST_STATE_INITIALIZATION_HPP