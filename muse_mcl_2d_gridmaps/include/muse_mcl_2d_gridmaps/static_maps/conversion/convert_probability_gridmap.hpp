#ifndef CONVERT_PROBABILITY_GRIDMAP_HPP
#define CONVERT_PROBABILITY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/utility/log_odds.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 ProbabilityGridMap::Ptr &dst)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new ProbabilityGridMap(origin,
                                     src.info.resolution,
                                     src.info.height,
                                     src.info.width,
                                     src.header.frame_id));

    const int8_t *occupancy_grid_ptr = src.data.data();
    const std::size_t size = dst->getHeight() * dst->getWidth();
    for(std::size_t i = 0 ; i < size ; ++i) {
        const int8_t p = occupancy_grid_ptr[i];
        if(p != -1) {
            dst->at(i) = p * 0.01;
        }
    }
}

inline void logOdds(ProbabilityGridMap::Ptr &src,
                    ProbabilityGridMap::Ptr &dst)
{
    if(src != dst) {
        dst.reset(new ProbabilityGridMap(*src));
    }
    std::for_each(dst->getData().begin(),
                  dst->getData().end(),
                  [](const double p){return utility::LogOdds::to(p);});

}
}
}
}

#endif // CONVERT_PROBABILITY_GRIDMAP_HPP
