#ifndef CONVERT_BINARY_GRIDMAP_HPP
#define CONVERT_BINARY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline BinaryGridMap::Ptr from(const nav_msgs::OccupancyGrid &occupancy_grid,
                               const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(occupancy_grid.info.origin.position.x,
                                     occupancy_grid.info.origin.position.y,
                                     tf::getYaw(occupancy_grid.info.origin.orientation));

    BinaryGridMap::Ptr map(new BinaryGridMap(origin,
                                             occupancy_grid.info.resolution,
                                             occupancy_grid.info.height,
                                             occupancy_grid.info.width,
                                             occupancy_grid.header.frame_id));

    const int8_t  t = threshold * 100;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    const std::size_t size = map->getHeight() * map->getWidth();
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        if(occupancy == -1) {
            occupancy = 50;
        }
        map->at(i) = occupancy >= t ? 1 : 0;
    }

    return map;
}
}
}
}

#endif // CONVERT_BINARY_GRIDMAP_HPP
