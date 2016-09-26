#include "distance_gridmap.h"
#include <tf/tf.h>

using namespace muse;
using namespace maps;

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                                 const double threshold) :
    GridMap<double>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width)
{
    convert(occupancy_grid, threshold);
}

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                                 const double threshold) :
    DistanceGridMap(*occupancy_grid, threshold)
{
}


void DistanceGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid, const double threshold)
{
    /// don't forget to count the resolution in ...
}
