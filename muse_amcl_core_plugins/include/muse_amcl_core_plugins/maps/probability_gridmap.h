#pragma once

#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_amcl {
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

