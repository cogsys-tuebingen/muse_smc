#ifndef MUSE_SMC_CYCLE_SCHEDULER_HPP
#define MUSE_SMC_CYCLE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>

namespace muse_smc {
template<typename state_space_description_t>
class CycleScheduler : public Scheduler<state_space_description_t>
{
public:
    using cycle_t       = std::size_t;
    using cycle_map_t   = std::unordered_map<id_t, cycle_t>;
    using update_t      = Update<state_space_description_t>;
    using resampling_t  = Resampling<state_space_description_t>;
    using sample_set_t  = SampleSet<state_space_description_t>;
    using Ptr           = std::shared_ptr<CycleScheduler>;

    CycleScheduler() :
        updates_applied_(0)
    {
    }

    void setup(const std::size_t &resampling_cycle_period,
               const cycle_map_t &update_cycle_periods)
    {
        resampling_cycle_period_ = resampling_cycle_period;
        update_cycle_periods_ = update_cycle_periods;
        for(const auto &u : update_cycle_periods_) {
            update_cycles_[u.first] = 0;
        }
    }

    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        assert(update_cycles_.find(u->getModelId()) != update_cycles_.end());
        assert(update_cycle_periods_.find(u->getModelId()) != update_cycle_periods_.end());

        const id_t id                     = u->getModelId();
        const cycle_t update_cycle_period = update_cycle_periods_[id];
        cycle_t &update_cycles            = update_cycles_[id];
        cycle_t &updates_applied          = updates_applied_;

        auto do_apply = [&u, &s, &updates_applied, &update_cycles] ()
        {
            u->apply(s->getWeightIterator());
            update_cycles = 1;
            ++updates_applied;
            return true;
        };
        auto do_not_apply = [&update_cycles] ()
        {
            ++update_cycles;
            return false;
        };

        return update_cycles > update_cycle_period ? do_apply() : do_not_apply();
    }

    virtual  bool apply(typename resampling_t::Ptr &r,
                        typename sample_set_t::Ptr &s) override
    {
        std::size_t &updates_applied = updates_applied_;
        auto do_apply = [&r,&s, &updates_applied](){
            r->apply(*s);
            updates_applied = 0;
            return true;
        };
        auto do_not_apply = [](){
            return false;
        };

        return updates_applied_ > resampling_cycle_period_ ? do_apply() : do_not_apply();
    }

protected:
    cycle_t     resampling_cycle_period_;
    cycle_t     updates_applied_;
    cycle_map_t update_cycles_;
    cycle_map_t update_cycle_periods_;
};
}

#endif // MUSE_SMC_CYCLE_SCHEDULER_HPP
