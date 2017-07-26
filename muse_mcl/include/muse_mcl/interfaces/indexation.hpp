#ifndef INDEXATION_HPP
#define INDEXATION_HPP

#include <muse_mcl/math/index.hpp>
#include "particle.hpp"

namespace muse_mcl {
template<std::size_t Dimension, typename StateT>
class Indexation {
public:
    using resolution_t  = std::array<double, Dimension>;
    using index_t       = muse_mcl::math::Index<Dimension>;
    using size_t        = std::array<std::size_t, Dimension>;
    using particle_t    = Particle<StateT>;
    using state_space_t = std::array<double, Dimension>;

    inline Indexation()
    {
        resolution_t.fill(0.0);
    }

    inline Indexation(const resolution_t &resolution) :
        resolution_(resolution)
    {
    }

    inline void setResolution(const resolution_t &resolution)
    {
        resolution_ = resolution;
    }

    inline const resolution_t & getResolution() const
    {
        return resolution_;
    }

    inline index_t create(const particle_t &sample)
    {
        return create(sample.state);
    }

    inline size_t getSize(const index_t &min,
                          const index_t &max)
    {
        size_t size;
        for(std::size_t d = 0ul ; d < Dimension ; ++d) {
            size[d] = static_cast<std::size_t>(max[d] - min[d]) + 1ul;
        }
    }

    virtual inline size_t getSize(state_space_t &state_space) = 0;
    virtual inline index_t create(const StateT &state) = 0;

private:
    resolution_t resolution_;
};
}
#endif // INDEXATION_HPP
