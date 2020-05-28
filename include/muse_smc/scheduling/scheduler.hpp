#ifndef MUSE_SMC_SCHEDULING_HPP
#define MUSE_SMC_SCHEDULING_HPP

#include <memory>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/update/update.hpp>

namespace muse_smc {
template <typename SampleSet_T, typename Update_T, typename Resampling_T>
class Scheduler {
 public:
  virtual bool apply(std::shared_ptr<Update_T> &u,
                     std::shared_ptr<SampleSet_T> &s) = 0;

  virtual bool apply(std::shared_ptr<Resampling_T> &r,
                     std::shared_ptr<SampleSet_T> &s) = 0;

 protected:
  Scheduler() = default;
  virtual ~Scheduler() = default;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SCHEDULING_HPP
