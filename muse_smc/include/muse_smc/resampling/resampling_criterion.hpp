#ifndef RESAMPLING_CRITERION_HPP
#define RESAMPLING_CRITERION_HPP

#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template<typename sample_t>
class ResamplingCriterion
{
public:
    using Ptr = std::shared_ptr<ResamplingCriterion>;
    using prediction_model_t = PredictionModel<sample_t>;

    ResamplingCriterion() = default;
    virtual ~ResamplingCriterion() = default;

    virtual void prediction(prediction_model_t::Result::Ptr &step) = 0;
    virtual void update() = 0;
    virtual void resample() = 0;

};
}

#endif // RESAMPLING_CRITERION_HPP
