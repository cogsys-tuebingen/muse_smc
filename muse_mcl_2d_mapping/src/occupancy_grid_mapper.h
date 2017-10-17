#ifndef OCCUPANCY_GRID_MAPPER_H
#define OCCUPANCY_GRID_MAPPER_H

#include <muse_mcl_2d_gridmaps/dynamic_maps/probability_gridmap.h>
#include <muse_mcl_2d_gridmaps/mapping/inverse_model.hpp>

#include "mapper.h"

namespace muse_mcl_2d_mapping {
class OccupancyGridMapper : public Mapper
{
public:
    using Ptr           = std::shared_ptr<OccupancyGridMapper>;
    using map_t         = muse_mcl_2d_gridmaps::dynamic_maps::ProbabilityGridMap;

    OccupancyGridMapper(const muse_mcl_2d_gridmaps::mapping::InverseModel &inverse_model,
                        const double resolution,
                        const double chunk_resolution,
                        const std::string &frame_id);



protected:
    virtual void process(const Pointcloud2D::Ptr &points) override;

    muse_mcl_2d_gridmaps::dynamic_maps::ProbabilityGridMap::Ptr map_;
    muse_mcl_2d_gridmaps::mapping::InverseModel                 inverse_model_;
    double                                                      resolution_;
    double                                                      chunk_resolution_;
    std::string                                                 frame_id_;

};
}

#endif // OCCUPANCY_GRID_MAPPER_H
