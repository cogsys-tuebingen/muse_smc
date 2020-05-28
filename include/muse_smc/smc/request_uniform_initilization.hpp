#ifndef MUSE_SMC_REQUEST_UNIFORM_INITIALIZATION_HPP
#define MUSE_SMC_REQUEST_UNIFORM_INITIALIZATION_HPP

namespace muse_smc {
template <typename Time_T>
class RequestUniformInitialization {
 public:
  inline explicit RequestUniformInitialization(const Time_T &time) : time_{time} {}

  const Time_T &time() const { return time_; }

 private:
  const Time_T time_;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_REQUEST_UNIFORM_INITIALIZATION_HPP