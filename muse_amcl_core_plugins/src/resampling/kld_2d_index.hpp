#ifndef KLD_2D_INDEX_HPP
#define KLD_2D_INDEX_HPP

#include <muse_amcl/math/pose.hpp>
#include <array>

namespace muse_amcl {
struct Index {
    using Type       = std::array<int, 3>;
    using PivotType  = double;
    static constexpr std::size_t Dimension = 3;

    Index(const double resolution_linear,
          const double resolution_radial) :
        resolution_linear(resolution_linear),
        resolution_radial(resolution_radial)
    {
    }

    inline Type create(const math::Pose &p)
    {
        return {
            static_cast<int>(floor(p.x() / resolution_linear)),
            static_cast<int>(floor(p.y() / resolution_linear)),
            static_cast<int>(floor(p.yaw()) / resolution_radial)
        };
    }

    const double resolution_linear;
    const double resolution_radial;

};
}

#endif // KLD_2D_INDEX_HPP
