#include "probability_gridmap.h"

using namespace muse;
using namespace maps;

ProbabilityGridMap::ProbabilityGridMap(const double origin_x,
                                       const double origin_y,
                                       const double origin_phi,
                                       const double resolution) :
    GridMap(origin_x, origin_y, origin_phi, resolution)
{
}

ProbabilityGridMap::ProbabilityGridMap(const tf::Pose &origin,
                                       const double resolution) :
    GridMap(origin, resolution)
{
}
