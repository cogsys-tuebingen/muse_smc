#ifndef CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
#define CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/likelihood_field_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline LikelihoodFieldGridMap::Ptr from(const nav_msgs::OccupancyGrid &occupancy_grid,
                                        const double maximum_distance = 2.0,
                                        const double sigma_hit = 0.5,
                                        const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);
    const double exp_factor_hit = (0.5 * 1.0 / (sigma_hit * sigma_hit));

    muse_mcl_2d::math::Pose2D origin(occupancy_grid.info.origin.position.x,
                                     occupancy_grid.info.origin.position.y,
                                     tf::getYaw(occupancy_grid.info.origin.orientation));

    LikelihoodFieldGridMap::Ptr map(new LikelihoodFieldGridMap(origin,
                                                               occupancy_grid.info.resolution,
                                                               occupancy_grid.info.height,
                                                               occupancy_grid.info.width,
                                                               maximum_distance,
                                                               sigma_hit,
                                                               occupancy_grid.header.frame_id));
    std::vector<int8_t> occ;
    occ.reserve(occupancy_grid.data.size());
    std::transform(occupancy_grid.data.begin(),
                   occupancy_grid.data.end(),
                   std::back_inserter(occ),
                   [](const int8_t p){return p != -1 ? p : 50;});

    /// 1.) calculate the distances
    algorithms::DistanceTransform<int8_t> distance_transform(occupancy_grid.info.resolution,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));
    distance_transform.apply(occ,
                             map->getWidth(),
                             map->getData());

    /// 2.) pre-calculation of the hit likelihoods
    std::for_each(map->getData().begin(),
                  map->getData().end(),
                  [exp_factor_hit] (const double z) {return std::exp(-z * z * exp_factor_hit);});
    return map;
}
}
}
}

#endif // CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
