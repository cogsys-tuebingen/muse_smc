#include <muse_mcl_2d_gridmaps/static_maps/distance_gridmap.h>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>


using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;
using namespace static_maps;

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                                 const double maximum_distance,
                                 const double threshold) :
    GridMap<double>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    occupancy_grid.header.frame_id)
{
    convert(occupancy_grid, threshold, maximum_distance);
}

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid, const double maximum_distance,
                                 const double threshold) :
    DistanceGridMap(*occupancy_grid, threshold)
{
}

double DistanceGridMap::at(const muse_mcl_2d::Point2D &point) const
{
    index_t i;
    toIndex(point, i);
    if(invalid(i))
        return std::numeric_limits<double>::max();
    return GridMap<double>::at(i[0], i[1]);
}



void DistanceGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                              const double threshold,
                              const double maximum_distance)
{
    algorithms::DistanceTransform<int8_t> distance_transform(resolution_,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));

    distance_transform.apply(occupancy_grid.data,
                             occupancy_grid.info.width,
                             data_);
}
