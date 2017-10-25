#ifndef STATIC_PROBABILITY_GRIDMAP_H
#define STATIC_PROBABILITY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/static_maps/gridmap.hpp>
#include <nav_msgs/OccupancyGrid.h>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
class ProbabilityGridmap : public Gridmap<double>
{
public:
    using Ptr = std::shared_ptr<ProbabilityGridmap>;

    ProbabilityGridmap(const pose_t &origin,
                       const double resolution,
                       const std::size_t height,
                       const std::size_t width,
                       const std::string &frame_id,
                       const double default_value = 0.5);

    ProbabilityGridmap(const ProbabilityGridmap &other) = default;
};
}
}

#endif /* STATIC_PROBABILITY_GRIDMAP_H */
