#ifndef CONVERT_DISTANCE_GRIDMAP_HPP
#define CONVERT_DISTANCE_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/distance_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {


inline DistanceGridMap::Ptr from(const nav_msgs::OccupancyGrid &occupancy_grid,
                                  const double threshold = 1.0,
                                  const double maximum_distance = 2.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(occupancy_grid.info.origin.position.x,
                                     occupancy_grid.info.origin.position.y,
                                     tf::getYaw(occupancy_grid.info.origin.orientation));

    DistanceGridMap::Ptr map(new DistanceGridMap(origin,
                                                 occupancy_grid.info.resolution,
                                                 maximum_distance,
                                                 occupancy_grid.info.height,
                                                 occupancy_grid.info.width,
                                                 occupancy_grid.header.frame_id));

    const int8_t  t = threshold * 100;
    std::vector<int8_t> occ;
    occ.reserve(occupancy_grid.data.size());
    std::transform(occupancy_grid.data.begin(),
                   occupancy_grid.data.end(),
                   std::back_inserter(occ),
                   [](const int8_t p){return p != -1 ? p : 50;});


    algorithms::DistanceTransform<int8_t> distance_transform(occupancy_grid.info.resolution,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));
    distance_transform.apply(occ,
                             map->getWidth(),
                             map->getData());

    return map;
}

}
}
}

#endif // CONVERT_DISTANCE_GRIDMAP_HPP
