#ifndef OCCUPANCY_GRID_MAPPER_H
#define OCCUPANCY_GRID_MAPPER_H

#include <muse_mcl_2d_mapping/pointcloud_2d.hpp>

namespace muse_mcl_2d_mapping {
class OccupancyGridMapper
{
public:
    OccupancyGridMapper();

    void insert(const Pointcloud2D::Ptr &points);

};
}

#endif // OCCUPANCY_GRID_MAPPER_H
