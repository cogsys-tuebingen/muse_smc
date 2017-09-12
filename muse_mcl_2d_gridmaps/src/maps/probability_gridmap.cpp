#include <muse_mcl_2d_gridmaps/maps/probability_gridmap.h>
#include <tf/tf.h>

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;
using namespace static_maps;

ProbabilityGridMap::ProbabilityGridMap(const nav_msgs::OccupancyGrid &occupancy_grid) :
    GridMap<double>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    occupancy_grid.header.frame_id)
{
    convert(occupancy_grid);
}

ProbabilityGridMap::ProbabilityGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid) :
    ProbabilityGridMap(*occupancy_grid)
{
}

void ProbabilityGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid)
{
    const std::size_t size = height_ * width_;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    data_.resize(size, 0);
    data_ptr_ = data_.data();
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        assert(occupancy <= 100);
        assert(occupancy >= -1);

        if(occupancy == -1) {
            data_ptr_[i] = 0.5;
        } else {
            data_ptr_[i] = occupancy / 100.0;
        }
    }
}
