#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>
#include <cslibs_time/statistics/duration.hpp>

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
    using mean_duration_t     = cslibs_time::statistics::Duration;
    using mean_duration_map_t = std::unordered_map<id_t, mean_duration_t>;
    using time_t              = cslibs_time::Time;
    using time_table_t        = std::unordered_map<id_t, time_t>;
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
        resampling_period_ = duration_t(1.0 / rate.expectedCycleTime().seconds());

        /// normalize the weights
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            time_slice_updates_[p.first]   = duration_t(resampling_period_ * (p.second / w));
            mean_durations_[p.first]       = mean_duration_t();
            time_slices_[p.first]          = duration_t();
        }
    }


    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        assert(mean_durations_.find(u->getModelId()) != mean_durations_.end());
        assert(time_slice_updates_.find(u->getModelId()) != time_slice_updates_.end());

        const time_t stamp = s->getStamp();
        const id_t id = u->getModelId();
        if(model_update_times_[id] < stamp) {
            model_update_times_[id] = stamp + offsets_[id];

            mean_duration_t &expected_duration = mean_durations_[id];
            time_slice_t &time_slice = time_slices_[id];
            if(expected_duration.mean() <= time_slice) {
#ifdef MUSE_SMC_DEBUG
                std::cerr << "model   " << u->getModelName() << std::endl;
                std::cerr << "offset: " << offsets_[id] << std::endl;
                std::cerr << "slice:  " << time_slice   << std::endl;
#endif
                const time_t start = time_t::now();
                u->apply(s->getWeightIterator());
                const duration_t dur_per_particle = (time_t::now() - start) / static_cast<double>(s->getSampleSize());
                expected_duration                += dur_per_particle;
                time_slice                       -= dur_per_particle;
                return true;
            }
        }
        return false;
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        auto do_apply = [&stamp, &r, &s, this] () {
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;


            const double sample_size = 1.0 / static_cast<double>(s->getSampleSize());
            for(const auto &ts : time_slice_updates_) {
                const id_t        id = ts.first;
                const int64_t  slice = (ts.second * sample_size).nanoseconds();
                /// how often is our model to be called
                const int64_t     nsecs = mean_durations_[id].mean().nanoseconds();
                /// when should it be called, get the lin space
                /// give it the resources
                time_slices_[id] = slice;
                offsets_[id] = nsecs > 0 && slice > nsecs ? duration_t(resampling_period_.nanoseconds() / (slice / nsecs)) : duration_t();
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
    time_slice_map_t        time_slices_;
    time_slice_map_t        offsets_;
    time_table_t            model_update_times_;

};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
