#ifndef BEAM_MODEL_MLE_H
#define BEAM_MODEL_MLE_H

#include <muse_amcl/particle_filter/update.hpp>

#include <atomic>

#include "beam_model_parameter_estimator.h"

namespace muse_mcl {
class BeamModelMLE : public UpdateModel
{
public:
    BeamModelMLE();

    virtual void update(const Data::ConstPtr &data,
                        const Map::ConstPtr &map,
                        ParticleSet::Weights set) override;

protected:
    std::size_t                             max_beams_;

    BeamModelParameterEstimator::Ptr        parameter_estimator_mle_;
    BeamModelParameterEstimator::Parameters parameters_;
    bool                                    use_estimated_parameters_;
    bool                                    use_weights_for_estimation_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;




};
}
#endif // BEAM_MODEL_MLE_H
