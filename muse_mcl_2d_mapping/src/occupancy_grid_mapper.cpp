#include "occupancy_grid_mapper.h"

using namespace muse_mcl_2d_mapping;

OccupancyGridMapper::OccupancyGridMapper(const muse_mcl_2d_gridmaps::mapping::InverseModel &inverse_model,
                                         const double resolution,
                                         const double chunk_resolution,
                                         const std::string &frame_id) :
    inverse_model_(inverse_model),
    resolution_(resolution),
    chunk_resolution_(chunk_resolution),
    frame_id_(frame_id)
{
}

void OccupancyGridMapper::process(const Pointcloud2D::Ptr &points)
{
    if(!map_) {
        const muse_mcl_2d::math::Pose2D &p = points->getOrigin();
        map_.reset(muse_mcl_2d_gridmaps::dynamic_maps::ProbabilityGridMap(p.tx(), p.ty(), p.yaw(),
                                                                          inverse_model_,
                                                                          resolution_,
                                                                          chunk_resolution_,
                                                                          frame_id));
    }

}
