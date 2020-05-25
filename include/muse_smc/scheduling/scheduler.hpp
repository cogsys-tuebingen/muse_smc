#ifndef MUSE_SMC_SCHEDULING_HPP
#define MUSE_SMC_SCHEDULING_HPP

#include <cslibs_time/rate.hpp>
#include <memory>
#include <muse_smc/resampling/resampling.hpp>
#include <muse_smc/update/update.hpp>

namespace muse_smc {
template <typename SampleSet_T, typename Update_T, typename Resampling_T>
class Scheduler {
 public:
  using Ptr = std::shared_ptr<Scheduler>;
  using ConstPtr = std::shared_ptr<Scheduler const>;

  virtual bool apply(typename Update_T::Ptr &u,
                     typename SampleSet_T::Ptr &s) = 0;

  virtual bool apply(typename Resampling_T::Ptr &r,
                     typename sample_set_t::Ptr &s) = 0;

 protected:
  Scheduler() = default;
  virtual ~Scheduler() = default;
};
}  // namespace muse_smc

#endif  // MUSE_SMC_SCHEDULING_HPP
