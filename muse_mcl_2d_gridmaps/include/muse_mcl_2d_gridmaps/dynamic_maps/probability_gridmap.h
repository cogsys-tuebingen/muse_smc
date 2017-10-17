#ifndef PROBABILITY_GRIDMAP_H
#define PROBABILITY_GRIDMAP_H

#include <muse_mcl_2d_gridmaps/dynamic_maps/gridmap.hpp>

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
class ProbabilityGridMap : public GridMap<double>
{
public:
    ProbabilityGridMap(const double  origin_x, const double  origin_y, const double  origin_phi,
                       const double  resolution, const double  chunk_resolution,
                       const double  default_value,
                       const std::string &frame_id);
};
}
}

#endif // PROBABILITY_GRIDMAP_H
