#include <muse_mcl_2d_gridmaps/dynamic_maps/probability_gridmap.h>

using namespace muse_mcl_2d_gridmaps;
using namespace dynamic_maps;

ProbabilityGridMap::ProbabilityGridMap(const pose_t &origin,
                                       const double resolution,
                                       const double chunk_resolution,
                                       const double default_value,
                                       const std::string &frame_id) :
    GridMap<double>(origin,
                    resolution,
                    chunk_resolution,
                    default_value,
                    frame_id)
{
}
