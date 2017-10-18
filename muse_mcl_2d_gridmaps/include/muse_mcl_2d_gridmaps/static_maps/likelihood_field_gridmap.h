#ifndef STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
#define STATIC_LIKELIHOOD_FIELD_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class LikelihoodFieldGridMap : public GridMap<double>
{
public:
    LikelihoodFieldGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                           const double sigma_hit,
                           const double maximum_distance = 2.0,
                           const double threshold = 1.0);
    LikelihoodFieldGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                           const double sigma_hit,
                           const double maximum_distance = 2.0,
                           const double threshold = 1.0);

    double at(const muse_mcl_2d::math::Point2D &point) const override;

    double getZHit() const;
    double getSigmaHit() const;

private:
    const double sigma_hit_;
    const double exp_factor_hit_;
    const double maximum_distance_;

    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold);

};
}
}

#endif // STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
