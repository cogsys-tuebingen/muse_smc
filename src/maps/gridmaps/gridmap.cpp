#include "gridmap.h"

#include <cmath>

using namespace muse;
using namespace maps;

GridMap::GridMap(const double origin_x,
                 const double origin_y,
                 const double origin_phi,
                 const double resolution) :
    origin_x_(origin_x),
    origin_y_(origin_y),
    origin_phi_(origin_phi),
    cos_origin_phi_(cos(origin_phi)),
    sin_origin_phi_(sin(origin_phi)),
    resolution_(resolution)
{
}

GridMap::GridMap(const tf::Pose &origin,
                 const double resolution) :
    GridMap(origin.getOrigin().x(),
            origin.getOrigin().y(),
            tf::getYaw(origin.getRotation()),
            resolution)
{
}
