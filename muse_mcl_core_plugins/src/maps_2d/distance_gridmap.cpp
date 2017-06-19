#include <muse_mcl_core_plugins/maps_2d/distance_gridmap.h>
#include <muse_mcl_core_plugins/maps_2d/distance_transform.hpp>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>

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
    data_.resize(size, 0.0);
    data_ptr_ = data_.data();

    cv::Mat src(height_, width_, CV_8UC1, cv::Scalar(1));
    for(std::size_t i = 0 ; i < size ; ++i) {
        int8_t occupancy = occupancy_grid_ptr[i];
        assert(occupancy <= 100);
        assert(occupancy >= -1);

        if(occupancy == -1 || occupancy >= threshold) {
            src.at<uchar>(i) = 0;
        }
    }

    cv::Mat dst(height_, width_, CV_32FC1, cv::Scalar());
    cv::distanceTransform(src, dst, CV_DIST_L2, 5);

    for(std::size_t i = 0 ; i < size ; ++i) {
        data_ptr_[i] = dst.at<float>(i) * resolution_;
    }


}
