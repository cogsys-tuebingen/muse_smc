#ifndef SYSTEMATIC_HPP
#define SYSTEMATIC_HPP

/// SYSTEM
#include <vector>
/// FRAMEWORK
#include <muse_amcl/particle_filter/particle.hpp>

namespace muse_amcl {
namespace resampling {
namespace impl {
struct Systematic {
    static inline void apply(const std::vector<Particle> &src,
                             std::vector<Particle>       &dst,
                             const std::size_t size)
    {
        /// initalize particle new particle set
        dst.resize(size);

        /// prepare ordered sequence of random numbers
        std::vector<double> u(size);
        {
            math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();
            for(std::size_t i = 0 ; i < size ; ++i) {
                u[i] = (i + u_static) / size;
            }
        }
        /// draw samples
        {
            auto src_it = src.begin();
            auto dst_it = dst.begin();
            double cumsum_last = 0.0;
            double cumsum = src_it->weight_;

            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            for(auto &u_r : u) {
                while(!in_range(u_r)) {
                    ++src_it;
                    cumsum_last = cumsum;
                    cumsum += src_it->weight_;
                }

                *dst_it = *src_it;
                ++dst_it;
            }
        }
    }
};
}
}
}


#endif // SYSTEMATIC_HPP
