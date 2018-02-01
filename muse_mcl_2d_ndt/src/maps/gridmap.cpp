#include <muse_mcl_2d_ndt/maps/gridmap.h>

namespace muse_mcl_2d_ndt {
Gridmap::Gridmap(const cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr &map,
                 const std::string frame_id) :
    muse_mcl_2d::Map2D(frame_id),
    data_(map)
{
}

Gridmap::state_space_boundary_t Gridmap::getMin() const
{
    return data_->getMin();
}

Gridmap::state_space_boundary_t Gridmap::getMax() const
{
    return data_->getMax();
}

Gridmap::state_space_transform_t Gridmap::getOrigin() const
{
    return data_->getOrigin();  // TODO: initial origin?
}

bool Gridmap::validate(const cslibs_math_2d::Pose2d &p) const
{
    return true;
}

cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr& Gridmap::data()
{
    return data_;
}

cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr const& Gridmap::data() const
{
    return data_;
}
}
