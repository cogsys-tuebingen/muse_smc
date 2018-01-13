#ifndef MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_BEAM_MODEL_AMCL_H
#define MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_BEAM_MODEL_AMCL_H

#include <muse_mcl_2d/update/update_model_2d.hpp>
#include <cslibs_gridmaps/utility/inverse_model.hpp>

namespace muse_mcl_2d_ndt {
class OccupancyGridmapBeamModelAMCL : public muse_mcl_2d::UpdateModel2D
{
public:
    OccupancyGridmapBeamModelAMCL();

    virtual void apply(const data_t::ConstPtr         &data,
                       const state_space_t::ConstPtr  &map,
                       sample_set_t::weight_iterator_t set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_short_;
    double      z_max_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_exponent_hit_;
    double      lambda_short_;
    double      occupied_threshold_;
    cslibs_gridmaps::utility::InverseModel::Ptr inverse_model_;

    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif // MUSE_MCL_2D_NDT_OCCUPANCY_GRIDMAP_BEAM_MODEL_AMCL_H
