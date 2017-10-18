#ifndef STATIC_PROBABILITY_GRIDMAP_H
#define STATIC_PROBABILITY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class ProbabilityGridMap : public GridMap<double>
{
public:
    ProbabilityGridMap(const nav_msgs::OccupancyGrid &occupancy_grid);
    ProbabilityGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid);

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid);

};
}
}

#endif /* STATIC_PROBABILITY_GRIDMAP_H */
