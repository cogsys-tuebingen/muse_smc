#include <muse_amcl/maps/distance_gridmap.h>
#include <muse_amcl/maps/distance_transform.hpp>
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
                    occupancy_grid.info.width,
                    occupancy_grid.header.frame_id)
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
    std::size_t size = height * width;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    std::vector<double> buffer(size);
    double * buffer_ptr = buffer.data();
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        assert(occupancy <= 100);
        assert(occupancy >= -1);

        if(occupancy == -1) {
            buffer_ptr[i] = 0.5;
        } else {
            buffer_ptr[i] = occupancy / 100.0;
        }
    }

    distance_transform::Borgefors<double> bf(height, width, resolution, threshold, 5);
    bf.apply(buffer, data);
}
