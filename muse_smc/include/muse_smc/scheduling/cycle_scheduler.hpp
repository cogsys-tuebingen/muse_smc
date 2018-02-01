#ifndef MUSE_SMC_CYCLE_SCHEDULER_HPP
#define MUSE_SMC_CYCLE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>

namespace muse_smc {
template<typename state_space_description_t>
class CycleScheduler : public Scheduler<state_space_description_t>
{
public:
    using threshold_map_t         = std::unordered_map<id_t, std::size_t>;
    using update_t      = Update<state_space_description_t>;
    using resampling_t  = Resampling<state_space_description_t>;
    using sample_set_t  = SampleSet<state_space_description_t>;

    CycleScheduler(const std::size_t    &resampling_update_cycle_threshold,
                   const threshold_map_t          &update_cycle_thresholds) :
        resampling_update_cycle_threshold_(resampling_update_cycle_threshold),
        updates_applied_(0),
        update_cycle_thresholds_(update_cycle_thresholds)
    {
    }

    virtual bool apply(typename update_t::Ptr &u,
                       typename sample_set_t::Ptr &s) override
    {
        const id_t id = u->getModelId();
        std::size_t &update_cycles = update_cycles_[id];
        std::size_t &updates_applied = updates_applied_;

        auto do_apply = [&u, &s, &update_cycles, &updates_applied] ()
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

        return update_cycles > update_cycle_thresholds_[id] ? do_apply() : do_not_apply();
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

        return updates_applied_ > resampling_update_cycle_threshold_ ? do_apply() : do_not_apply();
    }

protected:
    std::size_t     resampling_update_cycle_threshold_;
    std::size_t     updates_applied_;
    threshold_map_t update_cycles_;
    threshold_map_t update_cycle_thresholds_;
};
}


#endif // MUSE_SMC_CYCLE_SCHEDULER_HPP
