#ifndef DYNAMIC_PROBABILITY_GRIDMAP_H
#define DYNAMIC_PROBABILITY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/dynamic_maps/gridmap.hpp>

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
class ProbabilityGridMap : public GridMap<double>
{
public:
    ProbabilityGridMap(const pose_t &origin,
                       const double resolution,
                       const double chunk_resolution,
                       const std::string &frame_id,
                       const double default_value = 0.5);
};
}
}

#endif // DYNAMIC_PROBABILITY_GRIDMAP_H
