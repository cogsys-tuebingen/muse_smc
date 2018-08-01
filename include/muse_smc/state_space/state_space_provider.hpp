#ifndef STATE_SPACE_PROVIDER_HPP
#define STATE_SPACE_PROVIDER_HPP

#include <muse_smc/state_space/state_space.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class StateSpaceProvider
{
public:
    using Ptr           = std::shared_ptr<StateSpaceProvider>;
    using state_space_t = StateSpace<state_space_description_t>;

    virtual ~StateSpaceProvider() = default;

    virtual const std::string getName() const = 0;
    virtual typename state_space_t::ConstPtr getStateSpace() const = 0;     /// can return a state space
    virtual void  waitForStateSpace() const {}                              /// wait for state space to be avaliable

};
}

#endif // STATE_SPACE_PROVIDER_HPP
