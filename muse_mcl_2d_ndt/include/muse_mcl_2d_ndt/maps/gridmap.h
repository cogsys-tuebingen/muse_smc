#ifndef MUSE_MCL_2D_NDT_GRIDMAP_H
#define MUSE_MCL_2D_NDT_GRIDMAP_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_ndt_2d/dynamic_maps/gridmap.hpp>

namespace muse_mcl_2d_ndt {
class Gridmap : public muse_mcl_2d::Map2D
{
public:
    Gridmap(const cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr &map,
            const std::string frame_id);

    state_space_boundary_t getMin() const override;
    state_space_boundary_t getMax() const override;
    state_space_transform_t getOrigin() const override;
    bool validate(const cslibs_math_2d::Pose2d &p) const override;
    cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr& data();
    cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr const& data() const;

private:
    cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr data_;
};
}

#endif // MUSE_MCL_2D_NDT_GRIDMAP_H
