#ifndef DISTANCE_GRIDMAP_H
#define DISTANCE_GRIDMAP_H

#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_amcl {
namespace maps {
class DistanceGridMap : public GridMap<double>
{
public:
    DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                    const double threshold = 1.0,
                    const std::size_t kernel_size = 5);
    DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                    const double threshold = 1.0,
                    const std::size_t kernel_size = 5);

    double at(const math::Point &point) const;

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold,
                 const std::size_t kernel_size);

};
}
}

#endif /* DISTANCE_GRIDMAP_H */
