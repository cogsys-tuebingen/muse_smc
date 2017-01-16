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

    using Resolution =  std::array<double, 3>;
    using Index =  std::array<int, 3>;

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
            static_cast<int>(std::floor(sample.pose_.x() / resolution[0] + 0.5)),
            static_cast<int>(std::floor(sample.pose_.y() / resolution[1] + 0.5)),
            static_cast<int>(std::floor(sample.pose_.yaw() / resolution[2] + 0.5)),
        };
    }

    const Resolution resolution;

};
}
}
#endif // CLUSTERING_INDEX_HPP
