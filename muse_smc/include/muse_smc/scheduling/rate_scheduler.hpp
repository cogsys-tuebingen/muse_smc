#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>

namespace muse_smc {
template<typename state_space_description_t>
class RateScheduler : public Scheduler<state_space_description_t>
{
public:
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using time_slice_map_t    = std::unordered_map<id_t, cslibs_time::Duration>;


    using time_map_t   = std::unordered_map<id_t, cslibs_time::Duration>;

    RateScheduler(const cslibs_time::Rate   &r,
                  const time_priority_map_t &priorities,
                  const time_slice_map_t    &initial_time_slices = time_slice_map_t()) :
        resampling_period_(r.expectedCycleTime())
    {
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            time_update_map_[p.first] = resampling_period_ * (p.second / w);
        }
        if(!initial_time_slices.empty()) {
            assert(initial_time_slices.size() == priorities.size());


        }


        /// normalize weights and calculate the time slices
        /// initialize accounting with suiting weights
    }

    virtual bool applyUpdate(update_t::Ptr &u, sample_set_t::Ptr &s) override
    {
        assert(mean_durations_.find(u->getModelId()) != mean_durations_.end());
        assert(time_resource_map_.find(u->getModelId()) != time_resource_map_.end());
        assert(time_update_map_.find(u->getModelId()) != time_update_map_.end());

        auto do_apply = [this]() {

        };
        auto do_not_apply = [this]() {

        };
    }

    virtual bool applyResampling(resampling_t::Ptr &r, sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        auto do_apply = [&stamp, &r, &s, this] () {
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;
            return true;
        };
        auto do_not_apply = [] () {
            return false;
        };
        return resampline_time_ < stamp ? do_apply() : do_not_apply();
    }

protected:
    cslibs_time::Time       resampline_time_;
    cslibs_time::Duration   resampling_period_;

    time_slice_map_t        mean_durations_;        /// track the mean duration per particle
    time_slice_map_t        time_resource_map_;     /// computation time available for each model
    time_slice_map_t        time_update_map_;
};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
