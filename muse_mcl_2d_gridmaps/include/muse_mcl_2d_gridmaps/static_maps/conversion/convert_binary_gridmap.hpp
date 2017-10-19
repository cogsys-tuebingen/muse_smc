#ifndef CONVERT_BINARY_GRIDMAP_HPP
#define CONVERT_BINARY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 BinaryGridMap::Ptr            &dst,
                 const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new BinaryGridMap(origin,
                                src.info.resolution,
                                src.info.height,
                                src.info.width,
                                src.header.frame_id));

    const int8_t  t = threshold * 100;
    const int8_t *occupancy_grid_ptr = src.data.data();
    const std::size_t size = dst->getHeight() * dst->getWidth();
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        if(occupancy == -1) {
            occupancy = 50;
        }
        dst->at(i) = occupancy >= t ? 1 : 0;
    }
}
}
}
}

#endif // CONVERT_BINARY_GRIDMAP_HPP
