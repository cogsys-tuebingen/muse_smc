#include <muse_amcl_core_plugins/maps_2d/distance_gridmap.h>
#include <muse_amcl_core_plugins/maps_2d/distance_transform.hpp>
#include <tf/tf.h>

using namespace muse_mcl;
using namespace maps;

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                                 const double threshold, const std::size_t kernel_size) :
    GridMap<double>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    occupancy_grid.header.frame_id)
{
    convert(occupancy_grid, threshold, kernel_size);
}

DistanceGridMap::DistanceGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                                 const double threshold, const std::size_t kernel_size) :
    DistanceGridMap(*occupancy_grid, threshold)
{
}

double DistanceGridMap::at(const math::Point &point) const
{
    Index i;
    toIndex(point, i);
    if(invalid(i))
        return std::numeric_limits<double>::max();
    return GridMap<double>::at(i[0], i[1]);
}



void DistanceGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid, const double threshold, const std::size_t kernel_size)
{
    const std::size_t size = height_ * width_;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    std::vector<double> buffer(size);
    double * buffer_ptr = buffer.data();
    data_.resize(size, 0.0);
    data_ptr_ = data_.data();

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

    distance_transform::Borgefors<double> bf(height_, width_, resolution_, threshold, kernel_size);
    bf.apply(buffer, data_);
}
