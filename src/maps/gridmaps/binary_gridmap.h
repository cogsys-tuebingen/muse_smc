#ifndef BINARYGRIDMAP_H
#define BINARYGRIDMAP_H

#include "gridmap.h"

namespace muse {
namespace maps {
class BinaryGridMap : public GridMap
{
public:
    BinaryGridMap(const double origin_x,
                  const double origin_y,
                  const double origin_phi,
                  const double resolution);

    BinaryGridMap(const tf::Pose &origin,
                  const double resolution);
};
}
}

#endif // BINARYGRIDMAP_H
