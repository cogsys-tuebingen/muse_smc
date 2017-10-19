#include <muse_mcl_2d_gridmaps/static_maps/probability_gridmap.h>
#include <tf/tf.h>

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;
using namespace static_maps;

ProbabilityGridMap::ProbabilityGridMap(const pose_t &origin,
                                       const double resolution,
                                       const std::size_t height,
                                       const std::size_t width,
                                       const std::string &frame_id) :
    GridMap<double>(origin,
                    resolution,
                    height,
                    width,
                    0,
                    frame_id)
{
}


