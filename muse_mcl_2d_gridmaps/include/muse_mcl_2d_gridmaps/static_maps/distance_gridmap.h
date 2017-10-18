#ifndef STATIC_DISTANCE_GRIDMAP_H
#define STATIC_DISTANCE_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class DistanceGridMap : public GridMap<double>
{
public:
    DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                    const double maximum_distance = 2.0,
                    const double threshold = 1.0);
    DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                    const double maximum_distance = 2.0,
                    const double threshold = 1.0);

    double at(const muse_mcl_2d::math::Point2D &point) const override;

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold,
                 const double maximum_distance);

};
}
}

#endif /* STATIC_DISTANCE_GRIDMAP_H */
