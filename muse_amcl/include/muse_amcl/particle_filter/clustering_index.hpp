#ifndef CLUSTERING_INDEX_HPP
#define CLUSTERING_INDEX_HPP

#include "particle.hpp"

#include <assert.h>
#include <array>
#include <algorithm>

namespace muse_amcl {
namespace clustering {
struct Indexation {
    // @todo: Abstract to arbitrary dimensions
    // !currently only 2D !

    using Resolution = std::array<double, 3>;
    using Index = std::array<int, 3>;
    using Size  = std::array<std::size_t, 3>;

    Indexation(const Resolution &resolution) :
        resolution(resolution)
    {
        assert(resolution[0] > 0.0);
        assert(resolution[1] > 0.0);
        assert(resolution[2] > 0.0);
    }

    inline Index create(const Particle &sample) const
    {
        return {
            static_cast<int>(std::floor(sample.pose_.x()   / resolution[0])),
            static_cast<int>(std::floor(sample.pose_.y()   / resolution[1])),
            static_cast<int>(std::floor(sample.pose_.yaw() / resolution[2])),
        };
    }

    inline Index create(const std::array<double, 3> &pt)
    {
        return {
            static_cast<int>(std::floor(pt[0] / resolution[0])),
            static_cast<int>(std::floor(pt[1] / resolution[1])),
            static_cast<int>(std::floor(pt[2] / resolution[2])),
        };
    }

    inline Size size(const Particle &max,
                     const Particle &min)
    {
        const Index imin = create(min);
        const Index imax = create(max);
        return {imax[0] - imin[0] + 1ul,
                imax[1] - imin[1] + 1ul,
                imax[2] - imin[2] + 1ul};
    }

    inline Size size(const std::array<double, 3> &min,
                     const std::array<double, 3> &max)
    {
        const Index imin = create(min);
        const Index imax = create(max);
        return {imax[0] - imin[0] + 1ul,
                imax[1] - imin[1] + 1ul,
                imax[2] - imin[2] + 1ul};
    }

    const Resolution resolution;

};
}
}
#endif // CLUSTERING_INDEX_HPP
