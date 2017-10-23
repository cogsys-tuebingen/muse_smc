#ifndef STATE_SPACE_2D_HPP
#define STATE_SPACE_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
struct StateSpaceDescription2D
{
    using sample_allocator_t     = std::allocator<Sample2D>;
    using sample_t               = Sample2D;
    using state_t                = muse_mcl_math_2d::Pose2D;
    using state_space_boundary_t = muse_mcl_math_2d::Point2D;
    using transform_t            = muse_mcl_math_2d::Transform2D;
    using covariance_t           = muse_mcl_math_2d::Covariance2D;
};
}


#endif // STATE_SPACE_2D_HPP
