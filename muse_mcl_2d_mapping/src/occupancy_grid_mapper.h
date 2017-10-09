#ifndef OCCUPANCY_GRID_MAPPER_H
#define OCCUPANCY_GRID_MAPPER_H

#include <muse_mcl_2d_mapping/pointcloud_2d.hpp>
#include <muse_mcl_2d_gridmaps/dynamic_maps/probability_gridmap.h>
#include <muse_smc/utility/synchronized_queue.hpp>

namespace muse_mcl_2d_mapping {
class OccupancyGridMapper
{
public:
    using Ptr = std::shared_ptr<OccupancyGridMapper>;

    OccupancyGridMapper();

    void insert(const Pointcloud2D::Ptr &points);

private:
    muse_smc::synchronized::queue<Pointcloud2D::Ptr>            q_;
    muse_mcl_2d_gridmaps::dynamic_maps::ProbabilityGridMap::Ptr map_;


};
}

#endif // OCCUPANCY_GRID_MAPPER_H
