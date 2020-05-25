#ifndef MUSE_SMC_STATE_SPACE_PROVIDER_HPP
#define MUSE_SMC_STATE_SPACE_PROVIDER_HPP

namespace muse_smc {
template <typename StateSpace_T>
class StateSpaceProvider {
 public:
  using Ptr = std::shared_ptr<StateSpaceProvider>;
  using ConstPtr = std::shared_ptr<const StateSpaceProvider>;

  virtual std::string const& getName() const = 0;
  virtual typename StateSpace_T::ConstPtr getStateSpace()
      const = 0;  /// can return a state space
  virtual void waitForStateSpace() const {
  }  /// wait for state space to be avaliable

 protected:
  StateSpaceProvider() = default;
  virtual ~StateSpaceProvider() = default;
};
}  // namespace muse_smc


#endif  // MUSE_SMC_STATE_SPACE_PROVIDER_HPP
