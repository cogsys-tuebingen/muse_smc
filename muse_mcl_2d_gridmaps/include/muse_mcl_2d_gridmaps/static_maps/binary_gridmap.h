#ifndef STATIC_BINARY_GRIDMAP_H
#define STATIC_BINARY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class BinaryGridMap : public GridMap<int>
{
public:
    using Ptr = std::shared_ptr<BinaryGridMap>;
    enum state_t {FREE = 0, OCCUPIED = 1};


    BinaryGridMap(const pose_t &origin,
                  const double resolution,
                  const std::size_t height,
                  const std::size_t width,
                  const std::string &frame_id,
                  const state_t default_value = FREE);

    double getRange(const muse_mcl_2d::math::Point2D &from,
                    muse_mcl_2d::math::Point2D &to) const;

    double getRange2(const muse_mcl_2d::math::Point2D &from,
                     muse_mcl_2d::math::Point2D &to) const;

    virtual bool validate(const muse_mcl_2d::math::Pose2D &p) const;

};
}
}

#endif /* STATIC_BINARY_GRIDMAP_H */
