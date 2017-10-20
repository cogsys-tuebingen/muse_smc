#ifndef CONVERT_DISTANCE_GRIDMAP_HPP
#define CONVERT_DISTANCE_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/distance_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 DistanceGridMap::Ptr &dst,
                 const double threshold = 1.0,
                 const double maximum_distance = 2.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new DistanceGridMap(origin,
                                  src.info.resolution,
                                  maximum_distance,
                                  src.info.height,
                                  src.info.width,
                                  src.header.frame_id,
                                  maximum_distance));

    std::vector<int8_t> occ(src.data.size());
    std::transform(src.data.begin(),
                   src.data.end(),
                   occ.begin(),
                   [](const int8_t p){return p != -1 ? p : 50;});


    algorithms::DistanceTransform<int8_t> distance_transform(src.info.resolution,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));
    distance_transform.apply(occ,
                             dst->getWidth(),
                             dst->getData());
}
}
}
}

#endif // CONVERT_DISTANCE_GRIDMAP_HPP
