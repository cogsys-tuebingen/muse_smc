#ifndef WOF_HPP
#define WOF_HPP

/// SYSTEM
#include <vector>
/// FRAMEWORK
#include <muse_amcl/math/random.hpp>
#include <muse_amcl/particle_filter/particle.hpp>

namespace muse_amcl {
namespace resampling {
namespace impl {
struct WOF {
    static inline void apply(const std::vector<Particle> &src,
                             std::vector<Particle>       &dst,
                             const double w_max,
                             const std::size_t size)
    {
        dst.resize(size);
        math::random::Uniform<1> rng(0.0, 1.0);
        double beta = 0.0;
        std::size_t index = (std::size_t(rng.get() * size)) % size;
        for(std::size_t i = 0 ; i < size ; ++i) {
            beta += 2 * w_max * rng.get();
            while (beta > src[index].weight_) {
                beta -= src[index].weight_;
                index = (index + 1) % size;
            }
            dst[i] = src[index];
        }

    }
};
}
}
}


#endif // WOF_HPP
