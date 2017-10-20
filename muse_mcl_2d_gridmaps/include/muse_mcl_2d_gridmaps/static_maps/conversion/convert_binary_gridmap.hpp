#ifndef CONVERT_BINARY_GRIDMAP_HPP
#define CONVERT_BINARY_GRIDMAP_HPP

#include <muse_mcl_2d_gridmaps/static_maps/binary_gridmap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 BinaryGridMap::Ptr            &dst,
                 const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    muse_mcl_2d::math::Pose2D origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new BinaryGridMap(origin,
                                src.info.resolution,
                                src.info.height,
                                src.info.width,
                                src.header.frame_id));

    const int8_t  t = threshold * 100;
    std::transform(src.data.begin(), src.data.end(),
                  dst->getData().begin(),
                  [t](const int8_t p){int8_t occ = p == -1 ? 50 : p;
                                      return occ >= t ? BinaryGridMap::OCCUPIED : BinaryGridMap::FREE;});
}

inline void from(const nav_msgs::OccupancyGrid::ConstPtr &src,
                 BinaryGridMap::Ptr                      &dst,
                 const double threshold = 1.0)
{
    from(*src, dst, threshold);
}
}
}
}

#endif // CONVERT_BINARY_GRIDMAP_HPP
