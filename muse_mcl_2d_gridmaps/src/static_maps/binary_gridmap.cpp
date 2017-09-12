#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>
#include <tf/tf.h>

using namespace muse_mcl_2d;
using namespace muse_mcl_2d_gridmaps;
using namespace static_maps;

BinaryGridMap::BinaryGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                             const double threshold) :
    GridMap<int8_t>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    0,
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

double BinaryGridMap::getRange(const muse_mcl_2d::Point2D &from,
                               muse_mcl_2d::Point2D &to) const
{
    line_iterator_t it = getLineIterator(from, to);
    while(!it.done()) {
        if(*it)
            break;
        ++it;
    }
    if(it.invalid())
        return -1.0;

    fromIndex({it.x(), it.y()}, to);
    return  std::sqrt((to - from).length2());
}

bool BinaryGridMap::validate(const muse_mcl_2d::Pose2D &p) const
{
    index_t index;
    if(toIndex(p.translation(), index))
        return at(index[0], index[1]) == 0;
    return false;
}

void BinaryGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                            const double threshold)
{
    const std::size_t size = height_ * width_;
    const int8_t *occupancy_grid_ptr = occupancy_grid.data.data();
    data_.resize(size);
    data_ptr_ = data_.data();
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
        data_ptr_[i] = probability >= threshold ? 1 : 0;
    }
}
