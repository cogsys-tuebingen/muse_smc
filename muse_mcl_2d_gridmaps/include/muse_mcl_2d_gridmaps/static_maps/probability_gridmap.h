#ifndef STATIC_PROBABILITY_GRIDMAP_H
#define STATIC_PROBABILITY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class ProbabilityGridMap : public GridMap<double>
{
public:
    using Ptr = std::shared_ptr<ProbabilityGridMap>;

    ProbabilityGridMap(const pose_t &origin,
                       const double resolution,
                       const std::size_t height,
                       const std::size_t width,
                       const std::string &frame_id);

};
}
}

#endif /* STATIC_PROBABILITY_GRIDMAP_H */
