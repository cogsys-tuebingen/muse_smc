#ifndef STATE_PUBLISHER_HPP
#define STATE_PUBLISHER_HPP

#include <muse_smc/samples/sample_set.hpp>

namespace muse_smc {
template<typename sample_t>
class FilterState
{
public:
    using Ptr = std::shared_ptr<FilterState>;
    using sample_set_t = SampleSet<sample_t>;

    virtual void publish(const sample_set_t::Ptr &sample_set) = 0;
};
}

#endif // STATE_PUBLISHER_HPP
