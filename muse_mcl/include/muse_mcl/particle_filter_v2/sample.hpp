#ifndef SAMPLE_HPP
#define SAMPLE_HPP

#include <memory>
#include <utility>

namespace muse_mcl {
template<typename StateT>
struct Sample
{
    using Ptr = std::shared_ptr<Sample<StateT>>;

    StateT state;
    double weight;

    inline Sample() :
        weight(1.0)
    {
    }

    inline Sample(const StateT &state,
                  const double weight) :
        state(state),
        weight(weight)
    {
    }

    inline Sample(const Sample &other) :
        state(other.state),
        weight(other.weight)
    {
    }

    inline Sample(Sample &&other) :
        state(std::move(other.state)),
        weight(other.weight)
    {
    }
};
}


#endif // SAMPLE_HPP
