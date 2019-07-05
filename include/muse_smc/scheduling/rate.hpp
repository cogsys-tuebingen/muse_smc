#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>

#include <cslibs_time/statistics/duration_mean.hpp>

#include <utility>

namespace muse_smc {
template<typename sample_t>
class Rate : public muse_smc::Scheduler<sample_t>
{
public:
    using Ptr                 = std::shared_ptr<Rate>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = muse_smc::Update<sample_t>;
    using queue_t             = __gnu_pbds::priority_queue<Entry, typename Entry::Greater, __gnu_pbds::rc_binomial_heap_tag>;
    using mean_duration_t     = cslibs_time::statistics::DurationMean;
    using mean_duration_map_t = std::unordered_map<id_t, mean_duration_t>;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using resampling_t        = muse_smc::Resampling<sample_t>;
    using sample_set_t        = muse_smc::SampleSet<sample_t>;
    using nice_map_t          = std::unordered_map<id_t, double>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;
    using data_t              = traits::Data<sample_t>::type;

    virtual ~Rate() = default;

    void setup(const cslibs_time::Rate  &rate)
    {
        assert(rate.expectedCycleTime().seconds() != 0.0);

        resampling_period_ = duration_t(rate.expectedCycleTime().seconds());
    }

    virtual bool apply(typename update_t::Ptr     &u,
                       typename sample_set_t::Ptr &s) override
    {
        auto now = []()
        {
            return time_t(ros::Time::now().toNSec());
        };

        if(stamp >= next_update_time_) {
            const time_t start = now();
            u->apply(s->getWeightIterator());
            const duration_t dur = (now() - start);
            next_update_time_ = stamp + dur;
            return true;
        }
        return false;
    }


    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();

        auto now = []()
        {
            return time_t(ros::Time::now().toNSec());
        };

        if(resampling_time_.isZero())
            resampling_time_ = stamp;

        auto do_apply = [&stamp, &r, &s, &now, this] () {
            const time_t start = now();
            r->apply(*s);
            const duration_t dur = (now() - start);

            resampling_time_   = stamp + resampling_period_;
            next_update_time_  = next_update_time_ + dur;
            return true;
        };

        auto do_not_apply = [] () {
            return false;
        };

        return resampling_time_ < stamp ? do_apply() : do_not_apply();
    }

protected:
    time_t              next_update_time_;
    time_t              resampling_time_;
    duration_t          resampling_period_;
};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
