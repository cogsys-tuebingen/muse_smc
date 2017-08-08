#ifndef SAMPLE_2D_HPP
#define SAMPLE_2D_HPP

#include <memory>

#include <muse_mcl_2d/math/pose_2d.hpp>
#include <muse_mcl_2d/math/covariance_2d.hpp>

namespace muse_mcl_2d {
struct Sample2D {
public:
    using Ptr              = std::shared_ptr<Sample2D>;
    using allocator_t      = std::allocator<Sample2D>;
    using state_t          = Pose2D;
    using state_covariance = Covariance2D;

    double weight;
    Pose2D state;

};
}


#endif // SAMPLE_2D_HPP
