#ifndef PROBABILITY_GRIDMAP_H
#define PROBABILITY_GRIDMAP_H

#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl {
namespace maps {
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
#endif /* PROBABILITY_GRIDMAP_H */
