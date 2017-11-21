#ifndef MEASUREMENT_2D_HPP
#define MEASUREMENT_2D_HPP

#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/pointcloud.hpp>
#include <cslibs_time/time.hpp>

namespace muse_mcl_2d_mapping {
struct Measurement2d {
    const cslibs_math_2d::Pointcloud2d::Ptr points;
    const cslibs_math_2d::Pose2d            origin;
    const cslibs_time::Time                 stamp;

    explicit Measurement2d(const cslibs_math_2d::Pointcloud2d::Ptr &points,
                         const cslibs_math_2d::Pose2d            &origin,
                         const cslibs_time::Time                 &stamp) :
        points(points),
        origin(origin),
        stamp(stamp)
    {
    }
};
}


#endif // MEASUREMENT_2D_HPP
