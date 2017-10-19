#ifndef STATIC_DISTANCE_GRIDMAP_H
#define STATIC_DISTANCE_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class DistanceGridMap : public GridMap<double>
{
public:
    DistanceGridMap(const pose_t &origin,
                    const double resolution,
                    const double maximum_distance,
                    const std::size_t height,
                    const std::size_t width,
                    const std::string &frame_id);

    double at(const muse_mcl_2d::math::Point2D &point) const override;
};
}
}

#endif /* STATIC_DISTANCE_GRIDMAP_H */
