#pragma once

#include <cslibs_math_3d/linear/pose.hpp>
#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_time/time.hpp>

namespace muse_mcl_3d_mapping {
struct Measurement3d
{
    using cloud_t = cslibs_math_3d::Pointcloud3d;
    using pose_t  = cslibs_math_3d::Pose3d;
    using time_t  = cslibs_time::Time;

    const typename cloud_t::Ptr points;
    const pose_t                origin;
    const time_t                stamp;

    explicit Measurement3d(
            const typename cloud_t::Ptr & points,
            const pose_t                & origin,
            const time_t                & stamp) :
        points(points),
        origin(origin),
        stamp(stamp)
    { }
};
}
