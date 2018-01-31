#ifndef MUSE_SMC_RATE_SCHEDULER_HPP
#define MUSE_SMC_RATE_SCHEDULER_HPP

#include <muse_smc/scheduling/scheduler.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class RateScheduler : public Scheduler<state_space_description_t>
{
public:

    virtual applyUpdate(Update::Ptr &u, SampleSet::Ptr &s) override
    {

    }

    virtual applyResampling(Resampling::Ptr &r, SampleSet::Ptr &s) override
    {

    }

protected:

};
}

#endif // MUSE_SMC_RATE_SCHEDULER_HPP
