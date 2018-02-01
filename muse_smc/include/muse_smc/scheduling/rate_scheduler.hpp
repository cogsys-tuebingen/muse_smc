#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <cslibs_math/statistics/mean.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class RateScheduler : public Scheduler<state_space_description_t>
{
public:
    using weight_map_t = std::unordered_map<id_t, double>;
    using time_map_t   = std::unordered_map<id_t, cslibs_time::Duration>;

    RateScheduler(const cslibs_time::Rate &r) :
    {
    }

    RateScheduler(const cslibs_time::Rate &r,
                  const weight_map_t &weights)
    {

    }

    virtual applyUpdate(update_t::Ptr &u, sample_set_t::Ptr &s) override
    {

    }

    virtual applyResampling(resampling_t::Ptr &r, sample_set_t::Ptr &s) override
    {

    }

protected:
    cslibs_time::Time       last_resampling_time_;
    cslibs_time::Duration   resampling_period_;

    /// mean time of model application
    /// computation time map
    /// last time applied map
    /// computation resource map
    ///


};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
