#ifndef SAMPLE_INDEXATION_HPP
#define SAMPLE_INDEXATION_HPP

#include <muse_mcl/particle_filter_v2/sample.hpp>
#include <muse_mcl/math/index.hpp>

namespace muse_mcl {
template<typename StateT, typename Dimension>
/**
 * @brief The SampleIndexation struct is an abstract template class which represents
 *        the interface for indexation of state spaces / sub state spaces.
 */
struct SampleIndexation {
    using Ptr          = std::shared_ptr<SampleIndexation<StateT, Dimension>>;
    using resolution_t = std::array<double, Dimension>;
    using index_t      = math::Index<Dimension>;
    using size_t       = std::array<std::size_t, Dimension>;

    const resolution_t resolution;

    SampleIndexation(const resolution_t resolution) :
        resolution(resolution)
    {
    }

    virtual index_t create(const Sample<StateT> &sample) = 0;
    virtual index_t create(const StateT &state) = 0;
};
}


#endif // SAMPLE_INDEXATION_HPP
