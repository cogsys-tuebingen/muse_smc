#include <muse_amcl_core_plugins/maps/binary_gridmap.h>
#include <tf/tf.h>

using namespace muse;
using namespace maps;

BinaryGridMap::BinaryGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                             const double threshold) :
    GridMap<int8_t>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    occupancy_grid.header.frame_id)
{
    convert(occupancy_grid, threshold);
}

BinaryGridMap::BinaryGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                             const double threshold) :
    BinaryGridMap(*occupancy_grid,
                  threshold)
{
}

void BinaryGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                            const double threshold)
{
    std::size_t size = height * width;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        assert(occupancy <= 100);
        assert(occupancy >= -1);
        double probability;
        if(occupancy == -1) {
            probability = 0.5;
        } else {
            probability = occupancy / 100.0;
        }
        data_ptr[i] = probability >= threshold ? 1 : 0;
    }
}
