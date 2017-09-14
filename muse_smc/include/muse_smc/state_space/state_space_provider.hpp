#ifndef STATE_SPACE_PROVIDER_HPP
#define STATE_SPACE_PROVIDER_HPP

#include <muse_smc/state_space/state_space.hpp>

namespace muse_smc {
template<typename sample_t>
class StateSpaceProvider
{
public:
    using Ptr = std::shared_ptr<StateSpaceProvider>;
    using state_space_t = StateSpace<sample_t>;

    StateSpaceProvider() = default;
    virtual ~StateSpaceProvider() = default;

    inline const static std::string Type()
    {
        return "muse_smc::StateSpaceProvider";
    }

    inline std::string getName() const
    {
        return name_;
    }

    inline void setName(const std::string &name)
    {
        name_ = name;
    }

    inline std::size_t getId() const
    {
        return id_;
    }

    inline void setId(const std::size_t id)
    {
        id_ = id;
    }

    virtual typename state_space_t::ConstPtr getStateSpace() const = 0;

protected:
    std::string  name_;
    std::size_t  id_;
};
}

#endif // STATE_SPACE_PROVIDER_HPP
