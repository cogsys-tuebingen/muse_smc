#ifndef STATIC_BINARY_GRIDMAP_H
#define STATIC_BINARY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class BinaryGridMap : public GridMap<int8_t>
{
public:
    using Ptr = std::shared_ptr<BinaryGridMap>;

    BinaryGridMap(const nav_msgs::OccupancyGrid &occupancy_grid,
                  const double threshold = 1.0);
    BinaryGridMap(const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid,
                  const double threshold = 1.0);

    double getRange(const muse_mcl_2d::math::Point2D &from,
                    muse_mcl_2d::math::Point2D &to) const;

    double getRange2(const muse_mcl_2d::math::Point2D &from,
                     muse_mcl_2d::math::Point2D &to) const;


    virtual bool validate(const muse_mcl_2d::math::Pose2D &p) const;

private:
    void convert(const nav_msgs::OccupancyGrid &occupancy_grid,
                 const double threshold);

};
}
}

#endif /* STATIC_BINARY_GRIDMAP_H */
