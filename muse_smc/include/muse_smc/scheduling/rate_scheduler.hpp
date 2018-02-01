#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>
#include <cslibs_time/mean_duration.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class RateScheduler : public Scheduler<state_space_description_t>
{
public:
    using Ptr                 = std::shared_ptr<RateScheduler>;
    using rate_t              = cslibs_time::Rate;
    using time_slice_t        = cslibs_time::Duration;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using time_slice_map_t    = std::unordered_map<id_t, time_slice_t>;
    using mean_duration_t     = cslibs_time::MeanDuration;
    using mean_duration_map_t = std::unordered_map<id_t, mean_duration_t>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_t            = Update<state_space_description_t>;
    using resampling_t        = Resampling<state_space_description_t>;
    using sample_set_t        = SampleSet<state_space_description_t>;

    RateScheduler()
    {
    }

    void setup(const cslibs_time::Rate   &rate,
               const time_priority_map_t &priorities)
    {
        assert(rate.expectedCycleTime().seconds() != 0.0);
        resampling_period_ = 1.0 / rate.cycleTime().seconds();

        /// normalize the weights
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            time_slice_updates_[p.first]   = resampling_period_ * (p.second / w);
            mean_durations_[p.first]       = mean_duration_t();
            time_resources_[p.first]       = duration_t();
        }
    }

    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        assert(mean_durations_.find(u->getModelId()) != mean_durations_.end());
        assert(time_resources_.find(u->getModelId()) != time_resources_.end());
        assert(time_slice_updates_.find(u->getModelId()) != time_slice_updates_.end());

        const id_t id = u->getModelId();
        mean_duration_t &mean_duration     = mean_durations_[id];
        time_slice_t    &time_resource     = time_resources_[id];
        const duration_t expected_duration = mean_duration.duration() * static_cast<double>(s->getSampleSize());

        auto do_apply = [&u, &s, &mean_duration, &time_resource]() {
            const time_t start = time_t::now();
            u->apply(s->getWeightIterator());
            const duration_t dur = time_t::now() - start;
            mean_duration += dur / static_cast<double>(s->getSampleSize());
            time_resource -= dur;
            return true;
        };
        auto do_not_apply = [this]() {
            return false;
        };

        return time_resource > expected_duration ? do_apply() : do_not_apply();
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        auto do_apply = [&stamp, &r, &s, this] () {
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;

            /// book the resources for the current period
            for(const auto &ts : time_slice_updates_) {
                time_resources_[ts.first] += ts.second;
            }
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

    mean_duration_map_t     mean_durations_;        /// track the mean duration per particle
    time_slice_map_t        time_slice_updates_;    /// computation time available for each model
    time_slice_map_t        time_resources_;
};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
