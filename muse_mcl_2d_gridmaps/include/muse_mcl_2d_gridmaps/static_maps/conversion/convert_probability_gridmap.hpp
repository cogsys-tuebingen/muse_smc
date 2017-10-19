#ifndef CONVERT_PROBABILITY_GRIDMAP_HPP
#define CONVERT_PROBABILITY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/utility/log_odds.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline ProbabilityGridMap::Ptr from(const nav_msgs::OccupancyGrid &occupancy_grid,
                                    const bool log_odds = false)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(occupancy_grid.info.origin.position.x,
                                     occupancy_grid.info.origin.position.y,
                                     tf::getYaw(occupancy_grid.info.origin.orientation));

    ProbabilityGridMap::Ptr map(new ProbabilityGridMap(origin,
                                                       occupancy_grid.info.resolution,
                                                       occupancy_grid.info.height,
                                                       occupancy_grid.info.width,
                                                       occupancy_grid.header.frame_id));

    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    const std::size_t size = map->getHeight() * map->getWidth();
    for(std::size_t i = 0 ; i < size ; ++i) {
        const int8_t p = occupancy_grid_ptr[i];
        if(p != -1) {
            map->at(i) = p * 0.01;
        }
    }

    if(log_odds) {
        std::for_each(map->getData().begin(),
                      map->getData().end(),
                      [](const double p){return utility::LogOdds::to(p);});
    }

    return map;
}




}
}
}

#endif // CONVERT_PROBABILITY_GRIDMAP_HPP
