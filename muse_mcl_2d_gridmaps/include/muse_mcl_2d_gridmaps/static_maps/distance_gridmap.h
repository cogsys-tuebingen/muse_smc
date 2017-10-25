#ifndef STATIC_DISTANCE_GRIDMAP_H
#define STATIC_DISTANCE_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class DistanceGridmap : public Gridmap<double>
{
public:
    DistanceGridmap(const pose_t &origin,
                    const double resolution,
                    const double maximum_distance,
                    const std::size_t height,
                    const std::size_t width,
                    const std::string &frame_id,
                    const double default_value = 2.0);

    DistanceGridmap(const DistanceGridmap &other) = default;

    double at(const cslibs_math_2d::Point2d &point) const override;
    double getMaximumDistance() const;

private:
    double maximum_distance_;

};
}
}

#endif /* STATIC_DISTANCE_GRIDMAP_H */
