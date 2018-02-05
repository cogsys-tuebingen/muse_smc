#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>
#include <cslibs_time/mean_duration.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class RateCycleScheduler : public Scheduler<state_space_description_t>
{
public:
    using Ptr                 = std::shared_ptr<RateCycleScheduler>;
    using rate_t              = cslibs_time::Rate;
    using time_slice_t        = cslibs_time::Duration;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using time_slice_map_t    = std::unordered_map<id_t, time_slice_t>;
    using mean_duration_t     = cslibs_time::MeanDuration;
    using mean_duration_map_t = std::unordered_map<id_t, mean_duration_t>;
    using sampling_map_t      = std::unordered_map<id_t, std::size_t>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using update_t            = Update<state_space_description_t>;
    using resampling_t        = Resampling<state_space_description_t>;
    using sample_set_t        = SampleSet<state_space_description_t>;

    RateCycleScheduler()
    {
    }

    void setup(const cslibs_time::Rate   &rate,
               const time_priority_map_t &priorities)
    {
        assert(rate.expectedCycleTime().seconds() != 0.0);
        resampling_period_ = duration_t(1.0 / rate.expectedCycleTime().seconds());

        /// normalize the weights
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            time_slice_updates_[p.first]   = duration_t(resampling_period_ * (p.second / w));
            mean_durations_[p.first]       = mean_duration_t();
            sampling_periods_[p.first]     = 1;
            sampling_cycles_[p.first]      = 0;
        }
    }


    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        assert(mean_durations_.find(u->getModelId()) != mean_durations_.end());
        assert(time_resources_.find(u->getModelId()) != time_resources_.end());
        assert(time_slice_updates_.find(u->getModelId()) != time_slice_updates_.end());

        /*
         * Adjust the distritubion of models over time ( ....... | ........ | ........ )
         */

        const id_t id = u->getModelId();
        std::size_t &sampling_cycle = sampling_cycles_[id];
        auto do_apply = [&u, &s, id, &sampling_cycle, this]() {
            const time_t start = time_t::now();
            u->apply(s->getWeightIterator());
            const duration_t dur = time_t::now() - start;
            const duration_t dur_per_particle = dur / static_cast<double>(s->getSampleSize());
            mean_durations_[id] += dur_per_particle;
            sampling_cycle = 0;
            return true;
        };
        auto do_not_apply = [this]() {
            return false;
        };
        ++sampling_cycle;

        return sampling_cycle == sampling_periods_[id] ? do_apply() : do_not_apply();
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        auto do_apply = [&stamp, &r, &s, this] () {
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;

            /// book the resources for the current period
            const double sample_size = 1.0 / static_cast<double>(s->getSampleSize());
            for(const auto &ts : time_slice_updates_) {
                const double mean_duration  = mean_durations_[ts.first].milliseconds();
                sampling_periods_[ts.first] = mean_duration == 0.0 ? 1 : static_cast<std::size_t>(resampling_period_.milliseconds() * sample_size / mean_duration);
                sampling_cycles_[ts.first]  = 0;
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

    sampling_map_t          sampling_periods_;
    sampling_map_t          sampling_cycles_;

};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
