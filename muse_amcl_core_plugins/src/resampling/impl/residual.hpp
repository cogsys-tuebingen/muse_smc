#ifndef RESIDUAL_HPP
#define RESIDUAL_HPP

/// SYSTEM
#include <vector>
/// FRAMEWORK
#include <muse_amcl/math/random.hpp>
#include <muse_amcl/particle_filter/particle.hpp>

namespace muse_amcl {
namespace resampling {
namespace impl {
struct Residual {
    static inline void apply(const std::vector<Particle> &src,
                             std::vector<Particle>       &dst,
                             const std::size_t            size)
    {
        /// initalize particle new particle set
        dst.resize(size);
        /// draw copies of each particle
        auto dst_it  = dst.begin();
        auto dst_end = dst.end();
        std::vector<double> u(size);
        std::vector<double> w_residual(size);
        double              n_w_residual = 0.0;
        {
            math::random::Uniform<1> rng(0.0, 1.0);
            double u_static = rng.get();
            for(std::size_t i = 0 ; i < size ; ++i) {
                const auto &sample = src[i];
                u[i] = (i + u_static) / size;
                std::size_t copies = std::floor(sample.weight_ * size);

                w_residual[i] = size * sample.weight_ - copies;
                n_w_residual += w_residual[i];

                for(std::size_t i = 0 ;
                    i < copies && dst_it != dst_end ;
                    ++i ,++dst_it) {
                    *dst_it = sample;
                }
            }
        }
        {
            auto u_it = u.begin();
            auto p_it = src.begin();
            auto w_it = w_residual.begin();

            double cumsum_last = 0.0;
            double cumsum = 0.0;
            auto in_range = [&cumsum, &cumsum_last] (double u)
            {
                return u >= cumsum_last && u < cumsum;
            };

            std::size_t left = std::distance(dst_it, dst_end);
            for(std::size_t i = 0 ; i < left ; ++i) {
                while(!in_range(*u_it)) {
                    ++p_it;
                    ++w_it;
                    cumsum_last = cumsum;
                    cumsum += *w_it / n_w_residual;
                }

                *dst_it = *p_it;
                ++dst_it;
                ++u_it;
            }
        }
    }
};
}
}
}

#endif // RESIDUAL_HPP
