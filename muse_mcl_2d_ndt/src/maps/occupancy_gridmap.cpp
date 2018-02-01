#include <muse_mcl_2d_ndt/maps/occupancy_gridmap.h>

namespace muse_mcl_2d_ndt {
OccupancyGridmap::OccupancyGridmap(const cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr &map,
                 const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

OccupancyGridmap::state_space_boundary_t OccupancyGridmap::getMin() const
{
    return data_->getMin();
}

OccupancyGridmap::state_space_boundary_t OccupancyGridmap::getMax() const
{
    return data_->getMax();
}

OccupancyGridmap::state_space_transform_t OccupancyGridmap::getOrigin() const
{
    return data_->getOrigin();  // TODO: initial origin?
}

bool OccupancyGridmap::validate(const cslibs_math_2d::Pose2d &p) const
{
    return true;
}

cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr& OccupancyGridmap::data()
{
    return data_;
}

cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr const& OccupancyGridmap::data() const
{
    return data_;
}
}
