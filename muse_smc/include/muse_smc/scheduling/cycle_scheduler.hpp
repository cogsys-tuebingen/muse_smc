#ifndef MUSE_SMC_CYCLE_SCHEDULER_HPP
#define MUSE_SMC_CYCLE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>
#include <unordered_map>

namespace muse_smc {
template<typename state_space_description_t>
class CycleScheduler : public Scheduler<state_space_description_t>
{
public:
    using map_t = std::unordered_map<id_t, std::size_t>;

    CycleScheduler(const std::size_t    &resampling_update_cycle_threshold,
                   const map_t          &update_cycle_thresholds) :
        resampling_update_cycle_threshold_(resampling_update_cycle_threshold),
        updates_applied_(0),
        update_cycle_thresholds_(update_cycle_thresholds)
    {
    }

    virtual bool apply(Update::Ptr &u, SampleSet::Ptr &s) override
    {
        const id_t id = u->getModelId();
        std::size_t &update_cycles = update_cycles_[id];
        std::size_t &updates_applied = updates_applied_;

        auto do_apply = [&u, &s, &update_cycles, &updates_applied] ()
        {
            u->apply(sample_set_->getWeightIterator());
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

    virtual  bool apply(Resampling::Ptr &r, SampleSet::Ptr &s) override
    {
        auto do_apply = [&r,&s](){
            r->apply(*s);
            updates_applied_ = 0;
        };
        auto do_not_apply = [](){
            return false;
        };

        return updates_applied_ > resampling_update_cycle_threshold_ ? do_apply() : do_not_apply();
    }

protected:
    std::size_t     resampling_update_cycle_threshold_;
    std::size_t     updates_applied_;
    map_t           update_cycles_;
    map_t           update_cycle_thresholds_;


};
}


#endif // MUSE_SMC_CYCLE_SCHEDULER_HPP
