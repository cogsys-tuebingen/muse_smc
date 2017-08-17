#ifndef STATE_SPACE_2D_HPP
#define STATE_SPACE_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/prediction/prediction_integral_2d.hpp>

namespace muse_mcl_2d {
struct StateSpace2D
{
    using sample_allocator_t     = std::allocator<Sample2D>;
    using sample_t               = Sample2D;
    using state_t                = Pose2D;
    using state_space_boundary_t = Point2D;
    using transform_t            = Transform2D;
    using covariance_t           = Covariance2D;
    using prediction_integral_t  = PredictionIntegral2D;
};
}


#endif // STATE_SPACE_2D_HPP
