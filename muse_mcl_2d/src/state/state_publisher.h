#ifndef STATE_PUBLISHER_H
#define STATE_PUBLISHER_H

#include <muse_smc/smc/smc_state.hpp>
#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
class StatePublisher : public muse_smc::SMCState<Sample2D>
{
public:
    using Ptr = std::shared_ptr<StatePublisher>;

    StatePublisher();

    virtual void publish(const typename sample_set_t::Ptr &sample_set) override
    {
    }
    virtual void publishIntermidiate(const typename sample_set_t::Ptr &sample_set) override
    {
    }
};
}

#endif // STATE_PUBLISHER_H
