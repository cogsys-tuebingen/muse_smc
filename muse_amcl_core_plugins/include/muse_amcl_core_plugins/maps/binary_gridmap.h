#pragma once

#include "gridmap.hpp"
#include <nav_msgs/OccupancyGrid.h>

namespace muse_amcl {
namespace maps {
class BinaryGridMap : public GridMap<int8_t>
{
public:
    BinaryGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                  const double threshold = 1.0);
    BinaryGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                  const double threshold = 1.0);

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold);

};
}
}
