#ifndef STATE_PUBLISHER_HPP
#define STATE_PUBLISHER_HPP

#include <muse_smc/samples/sample_set.hpp>
#include <muse_smc/time/time.hpp>

namespace muse_smc {
template<typename sample_t>
class SMCState
{
public:
    using Ptr = std::shared_ptr<SMCState>;
    using sample_set_t = SampleSet<sample_t>;

    virtual void publish(const typename sample_set_t::Ptr &sample_set) = 0;
    virtual void publishIntermidiate(const typename sample_set_t::Ptr &sample_set) = 0;

};
}

#endif // STATE_PUBLISHER_HPP
