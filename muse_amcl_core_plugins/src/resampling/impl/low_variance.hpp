#ifndef LOW_VARIANCE_HPP
#define LOW_VARIANCE_HPP

/// SYSTEM
#include <vector>
/// FRAMEWORK
#include <muse_amcl/particle_filter/particle.hpp>

namespace muse_amcl {
namespace resampling {
namespace impl {
struct LowVariance {
    static inline void apply(const std::vector<Particle> &src,
                             std::vector<Particle>       &dst,
                             const std::size_t = src.size())
    {
    }
};
}
}
}

#endif // LOW_VARIANCE_HPP
