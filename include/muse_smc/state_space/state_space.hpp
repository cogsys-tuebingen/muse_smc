#ifndef MUSE_SMC_STATE_SPACE_HPP
#define MUSE_SMC_STATE_SPACE_HPP

#include <chrono>
#include <cslibs_time/time.hpp>
#include <memory>
#include <muse_smc/smc/traits/sample.hpp>
namespace muse_smc {
template <typename State_T, typename StateSpaceTransform_T,
          typename StateSpaceBoundary_T>
class StateSpace {
 public:
  using Ptr = std::shared_ptr<StateSpace>;
  using ConstPtr = std::shared_ptr<StateSpace const>;

  inline explicit StateSpace(const std::string &frame) : frame_{frame} {}

  inline explicit StateSpace(const std::string &frame,
                             const cslibs_time::Time &stamp)
      : frame_{frame}, stamp_{stamp} {}

  virtual ~StateSpace() = default;

  virtual bool validate(const State_T &) const = 0;
  virtual StateSpaceBoundary_T getMin() const = 0;
  virtual StateSpaceBoundary_T getMax() const = 0;
  virtual StateSpaceTransform_T getOrigin() const = 0;

  inline std::string const &getFrame() const { return frame_; }

  inline cslibs_time::Time const &getStamp() const { return stamp_; }

  template <typename T>
  inline bool isType() const {
    const T *t = dynamic_cast<const T *>(this);
    return t != nullptr;
  }

  template <typename T>
  inline T const &as() const {
    return dynamic_cast<const T &>(*this);
  }

 protected:
  StateSpace() = delete;

  std::string frame_;
  cslibs_time::Time stamp_{cslibs_time::Time::now()};
};
}  // namespace muse_smc

#endif  // MUSE_SMC_STATE_SPACE_HPP
