#ifndef STATE_SPACE_2D_HPP
#define STATE_SPACE_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>

namespace muse_mcl_2d {
struct StateSpaceDescription2D
{
    using sample_allocator_t     = std::allocator<Sample2D>;
    using sample_t               = Sample2D;
    using state_t                = math::Pose2D;
    using state_space_boundary_t = math::Point2D;
    using transform_t            = math::Transform2D;
    using covariance_t           = math::Covariance2D;
};
}


#endif // STATE_SPACE_2D_HPP
