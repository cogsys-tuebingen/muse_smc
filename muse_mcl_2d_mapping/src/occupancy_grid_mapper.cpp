#include "occupancy_grid_mapper.h"

using namespace muse_mcl_2d_mapping;

OccupancyGridMapper::OccupancyGridMapper()
{
}


void OccupancyGridMapper::insert(const Pointcloud2D::Ptr &points)
{
    q_.emplace(points);
}
