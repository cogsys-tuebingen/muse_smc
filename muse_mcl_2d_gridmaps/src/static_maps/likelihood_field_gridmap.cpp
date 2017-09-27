#include <muse_mcl_2d_gridmaps/static_maps/likelihood_field_gridmap.h>

#include <muse_mcl_2d_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;
using namespace static_maps;

LikelihoodFieldGridMap::LikelihoodFieldGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                                               const double sigma_hit,
                                               const double maximum_distance,
                                               const double threshold) :
    GridMap<double>(occupancy_grid.info.origin.position.x,
                    occupancy_grid.info.origin.position.y,
                    tf::getYaw(occupancy_grid.info.origin.orientation),
                    occupancy_grid.info.resolution,
                    occupancy_grid.info.height,
                    occupancy_grid.info.width,
                    0.5,
                    occupancy_grid.header.frame_id),
    sigma_hit_(sigma_hit),
    exp_factor_hit_(0.5 * 1.0 / (sigma_hit * sigma_hit)),
    maximum_distance_(maximum_distance)

{
    convert(occupancy_grid, threshold);
}

LikelihoodFieldGridMap::LikelihoodFieldGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                                               const double sigma_hit,
                                               const double maximum_distance,
                                               const double threshold) :
    LikelihoodFieldGridMap(*occupancy_grid, sigma_hit,  maximum_distance, threshold)
{
}

double LikelihoodFieldGridMap::at(const muse_mcl_2d::math::Point2D &point) const
{
    index_t i;
    toIndex(point, i);
    if(invalid(i))
        return 0.0;
    return GridMap<double>::at(i[0], i[1]);
}

double LikelihoodFieldGridMap::getSigmaHit() const
{
    return sigma_hit_;
}

void LikelihoodFieldGridMap::convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                                     const double threshold)
{
    /// 1.) calculate the distances
    algorithms::DistanceTransform<int8_t> distance_transform(resolution_,
                                                             maximum_distance_,
                                                             static_cast<int8_t>(threshold * 100));

    std::vector<double> distances;
    distance_transform.apply(occupancy_grid.data,
                             occupancy_grid.info.width,
                             distances);


    /// 2.) pre-calculation of the hit likelihoods
    auto p_hit = [this] (const double z) {
        return std::exp(-z * z * exp_factor_hit_);
    };

    const std::size_t size = height_ * width_;
    data_.resize(size, 0);

    double       *data_ptr = data_.data();
    const double *distances_ptr = distances.data();
    for(std::size_t i = 0 ; i < size ; ++i, ++data_ptr, ++distances_ptr) {
        *data_ptr = p_hit(*distances_ptr);

    }
}
