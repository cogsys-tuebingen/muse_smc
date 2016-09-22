#include "binary_gridmap.h"

using namespace muse;
using namespace maps;

BinaryGridMap::BinaryGridMap(const double origin_x,
                             const double origin_y,
                             const double origin_phi,
                             const double resolution) :
    GridMap(origin_x, origin_y, origin_phi, resolution)
{
}

BinaryGridMap::BinaryGridMap(const tf::Pose &origin,
                             const double resolution) :
    GridMap(origin, resolution)
{
}
