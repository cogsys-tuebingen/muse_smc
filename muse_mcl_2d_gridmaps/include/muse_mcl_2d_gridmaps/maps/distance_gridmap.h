#ifndef DISTANCE_GRIDMAP_H
#define DISTANCE_GRIDMAP_H

#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl {
namespace maps {
class DistanceGridMap : public GridMap<double>
{
public:
    DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                    const double maximum_distance = 2.0,
                    const double threshold = 1.0);
    DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                    const double maximum_distance = 2.0,
                    const double threshold = 1.0);

    double at(const math::Point &point) const;

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold,
                 const double maximum_distance);

};
}
}

#endif /* DISTANCE_GRIDMAP_H */
