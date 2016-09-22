#ifndef DISTANCEGRIDMAP_H
#define DISTANCEGRIDMAP_H

#include "gridmap.h"

namespace muse {
namespace maps {
class DistanceGridMap : public GridMap
{
public:

    DistanceGridMap(const double origin_x,
                    const double origin_y,
                    const double origin_phi,
                    const double resolution);

    DistanceGridMap(const tf::Pose &origin,
                    const double resolution);
};
}
}

#endif // DISTANCEGRIDMAP_H
