#ifndef PROBABILITYGRIDMAP_H
#define PROBABILITYGRIDMAP_H

#include "gridmap.h"

namespace muse {
namespace maps {
class ProbabilityGridMap : public GridMap
{
public:
    ProbabilityGridMap(const double origin_x,
                       const double origin_y,
                       const double origin_phi,
                       const double resolution);

    ProbabilityGridMap(const tf::Pose &origin,
                       const double resolution);
};
}
}

#endif // PROBABILITYGRIDMAP_H
