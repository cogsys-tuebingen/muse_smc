#ifndef MUSE_SMC_CFS_RATE_HPP
#define MUSE_SMC_CFS_RATE_HPP

#include <ext/pb_ds/priority_queue.hpp>

#include <muse_smc/scheduling/scheduler.hpp>

#include <cslibs_time/statistics/duration.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class CFSRate : public Scheduler<state_space_description_t>
{
public:
    struct Entry {
        int64_t     vtime;
        std::size_t id;

        Entry(const std::size_t id) :
            vtime(0),
            id(id)
        {
        }

        Entry(const int64_t     vtime,
              const std::size_t id) :
            vtime(vtime),
            id(id)
        {
        }

        struct Greater {
            bool operator()( const Entry& lhs,
                             const Entry& rhs ) const
            {
                return lhs.vtime > rhs.vtime;
            }
        };
    };

    using Ptr                 = std::shared_ptr<CFSRate>;
    using rate_t              = cslibs_time::Rate;
    using update_t            = Update<state_space_description_t>;
    using queue_t             = __gnu_pbds::priority_queue<Entry, typename Entry::Greater, __gnu_pbds::rc_binomial_heap_tag>;
    using mean_duration_t     = cslibs_time::statistics::Duration;
    using mean_duration_map_t = std::unordered_map<id_t, mean_duration_t>;
    using time_priority_map_t = std::unordered_map<id_t, double>;
    using resampling_t        = Resampling<state_space_description_t>;
    using sample_set_t        = SampleSet<state_space_description_t>;
    using priority_map_t      = std::unordered_map<id_t, double>;
    using nice_map_t          = std::unordered_map<id_t, double>;
    using time_t              = cslibs_time::Time;
    using duration_t          = cslibs_time::Duration;


    void setup(const cslibs_time::Rate  &rate,
               const priority_map_t     &priorities)
    {
        assert(rate.expectedCycleTime().seconds() != 0.0);

        resampling_period_ = duration_t(rate.expectedCycleTime().seconds());

        /// normalize the weights
        double w = 0.0;
        for(const auto &p : priorities) {
            w += p.second;
        }
        for(const auto &p : priorities) {
            nice_values_[p.first]     = 1.0 - (p.second / w);
            mean_durations_[p.first] = mean_duration_t();
            q_.push(Entry(p.first));
        }
    }

    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        std::cerr << "was here" << std::endl;

        const id_t id = u->getModelId();
        const time_t stamp = u->getStamp();
        std::cerr << q_.top().id << " " << id << "\n";
        std::cerr << stamp << " " << next_update_time_ << "\n";

        if(id == q_.top().id && stamp >= next_update_time_) {
            Entry entry = q_.top();
            q_.pop();
            const time_t start = time_t::now();
            u->apply(s->getWeightIterator());
            const duration_t dur = (time_t::now() - start);
            entry.vtime += static_cast<int64_t>(dur.nanoseconds() * nice_values_[id]);
            next_update_time_ = u->getStamp() + dur;
            q_.push(entry);
            return true;
        }
        return false;
    }


    virtual bool apply(typename resampling_t::Ptr &r,
                       typename sample_set_t::Ptr &s) override
    {
        const cslibs_time::Time &stamp = s->getStamp();
        if(resampline_time_.isZero())
            resampline_time_ = stamp;

        auto do_apply = [&stamp, &r, &s, this] () {
            r->apply(*s);
            resampline_time_ = stamp + resampling_period_;

            int64_t min_vtime = q_.top().vtime;
            queue_t q;
            for(auto e : q_) {
                e.vtime -= min_vtime;
                q.push(e);
            }
            std::swap(q, q_);
            return true;
        };
        auto do_not_apply = [] () {
            return false;
        };
        return resampline_time_ < stamp ? do_apply() : do_not_apply();
    }
protected:
    cslibs_time::Time       next_update_time_;
    cslibs_time::Time       resampline_time_;
    cslibs_time::Duration   resampling_period_;
    mean_duration_map_t     mean_durations_;        /// track the mean duration per particle
    nice_map_t              nice_values_;
    queue_t                 q_;
};
}
#endif // MUSE_SMC_CFS_RATE_HPP
