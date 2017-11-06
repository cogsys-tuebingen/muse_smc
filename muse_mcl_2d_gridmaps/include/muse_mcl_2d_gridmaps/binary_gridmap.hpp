#ifndef MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP

#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_math/common/framed.hpp>
#include <cslibs_gridmaps/static_maps/binary_gridmap.h>


namespace muse_mcl_2d_gridmaps {
class BinaryGridmap : public muse_mcl_2d::Map2D
{
public:

    inline BinaryGridmap(const cslibs_gridmaps::static_maps::BinaryGridMap::Ptr &map,
                         const std::string frame_id) :
         muse_mcl_2d::Map2D(frame_id),
         data_(map)
    {
    }

    inline virtual state_space_boundary_t getMin() const override
    {
        return data_->getMin();
    }

    inline virtual state_space_boundary_t getMax() const override
    {
        return data_->getMax();
    }

    inline virtual state_space_transform_t getOrigin() const override
    {
        return data_->getOrigin();
    }

    inline cslibs_gridmaps::static_maps::BinaryGridMap::Ptr& data()
    {
        return data_;
    }

    inline const cslibs_gridmaps::static_maps::BinaryGridMap::Ptr& data() const
    {
        return data_;
    }

private:
    cslibs_gridmaps::static_maps::BinaryGridMap::Ptr data_;

};
}

#endif // MUSE_MCL_2D_GRIDMAPS_BINARY_GRIDMAP_HPP
