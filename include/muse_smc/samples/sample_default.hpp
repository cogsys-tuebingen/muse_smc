#ifndef SAMPLE_HPP
#define SAMPLE_HPP

#include <memory>
#include <utility>
#include <vector>

/**
 * Default sample type.
 */
namespace muse {
template<typename state_t>
struct Sample
{
    using Ptr              = std::shared_ptr<Sample<state_t>>;
    using allocator_t      = std::allocator<Sample<state_t>>;
    using state_t          = state_t;
    using transform_t      = state_t;
    using state_covariance = std::vector<double>;

    state_t state;
    double weight;

    inline Sample() :
        weight(1.0)
    {
    }

    inline Sample(const double weight) :
        weight(weight)
    {
    }

    inline Sample(const state_t &state,
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
