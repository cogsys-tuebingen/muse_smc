#ifndef MUSE_MCL_2D_GRIDMAP_3D_LIKELIHOOD_FIELD_MODEL_NORMALIZED_H
#define MUSE_MCL_2D_GRIDMAP_3D_LIKELIHOOD_FIELD_MODEL_NORMALIZED_H

#include <muse_mcl_2d/update/update_model_2d.hpp>

namespace muse_mcl_2d_ndt {
class Gridmap3dLikelihoodFieldModelNormalized : public muse_mcl_2d::UpdateModel2D
{
public:
    Gridmap3dLikelihoodFieldModelNormalized();

    virtual void apply(const data_t::ConstPtr         &data,
                       const state_space_t::ConstPtr  &map,
                       sample_set_t::weight_iterator_t set) override;

protected:
    std::size_t         max_points_;
    double              d1_;
    double              d2_;
    std::vector<double> ps_;

    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // MUSE_MCL_2D_GRIDMAP_3D_LIKELIHOOD_FIELD_MODEL_NORMALIZED_H
