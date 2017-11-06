#ifndef MUSE_MCL_2D_GRIDMAPS_LIKELIHOOD_FIELD_GRIDMAP_HPP
#define MUSE_MCL_2D_GRIDMAPS_LIKELIHOOD_FIELD_GRIDMAP_HPP
#include <muse_mcl_2d/map/map_2d.hpp>

#include <cslibs_time/stamped.hpp>
#include <cslibs_math/common/framed.hpp>
#include <cslibs_gridmaps/static_maps/likelihood_field_gridmap.h>

namespace muse_mcl_2d_gridmaps {
class LikelihoodFieldGridmap : public muse_mcl_2d::Map2D
{
public:
    inline LikelihoodFieldGridmap(const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr &map,
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

    inline cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr& data()
    {
        return data_;
    }

    inline const cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr& data() const
    {
        return data_;
    }


private:
    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr data_;

};
}
#endif // LIKELIHOOD_FIELD_GRIDMAP_HPP
