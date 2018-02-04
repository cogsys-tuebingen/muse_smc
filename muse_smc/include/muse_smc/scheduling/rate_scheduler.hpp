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
        resampling_period_ = duration_t(1.0 / rate.expectedCycleTime().seconds());

        /// normalize the weights
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            time_slice_updates_[p.first]   = duration_t(resampling_period_ * (p.second / w));
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
        const duration_t expected_duration = mean_duration.duration();

        /*
         * Adjust the distritubion of models over time ( ....... | ........ | ........ )
         */


        auto do_apply = [&u, &s, &mean_duration, &time_resource]() {
            const time_t start = time_t::now();
            u->apply(s->getWeightIterator());
            const duration_t dur = time_t::now() - start;
            const duration_t dur_per_particle = dur / static_cast<double>(s->getSampleSize());
            mean_duration += dur_per_particle;
            std::cerr << "      " << time_resource.milliseconds() << " " << dur_per_particle.milliseconds() << std::endl;
            time_resource -= dur_per_particle;
            std::cerr << "      " << dur.milliseconds() << std::endl;
            return true;
        };
        auto do_not_apply = [this]() {
            return false;
        };
        return time_resource >= time_duration() ? do_apply() : do_not_apply();
    }

    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        auto do_apply = [&stamp, &r, &s, this] () {
            std::cerr << "resampling: ----" << std::endl;
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;

            /// book the resources for the current period
            std::cerr << "booking: ---- " << time_slice_updates_.size() << std::endl;
            const double sample_size = 1.0 / static_cast<double>(s->getSampleSize());
            for(const auto &ts : time_slice_updates_) {
                auto dur_to_book = ts.second * sample_size;
                std::cerr << time_resources_[ts.first].milliseconds()  << " <- " << dur_to_book.milliseconds() << std::endl;
                time_resources_[ts.first] = ts.second * sample_size;
                std::cerr << time_resources_[ts.first].milliseconds() << std::endl;
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
