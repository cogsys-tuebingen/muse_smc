#ifndef BEAM_MODEL_H
#define BEAM_MODEL_H

#include <muse_mcl_2d/update/update_model_2d.hpp>

namespace muse_mcl_2d_gridmaps {
class BeamModel : public muse_mcl_2d::UpdateModel2D
{
public:
    BeamModel();

    virtual void apply(const data_t::ConstPtr        &data,
                       const state_space_t::ConstPtr &map,
                       sample_set_t::weight_iterator_t set) override;

protected:
    std::size_t max_beams_;
    double      z_hit_;
    double      z_short_;
    double      z_max_;
    double      z_rand_;
    double      sigma_hit_;
    double      denominator_exponent_hit_;
    double      denominator_hit_;
    double      lambda_short_;

    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* BEAM_MODEL_H */
