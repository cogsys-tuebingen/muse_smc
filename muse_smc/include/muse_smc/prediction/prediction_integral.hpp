#ifndef PREDICTION_INTEGRAL_HPP
#define PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template<typename sample_t, typename data_t>
class PredictionIntegral
{
public:
    using Ptr = std::shared_ptr<PredictionIntegral>;
    using prediction_model_t = PredictionModel<sample_t>;

    PredictionIntegral() = default;
    virtual ~PredictionIntegral() = default;

    virtual void add(prediction_model_t::Result::Ptr &step) = 0;
    virtual void reset() = 0;
    virtual data_t get() = 0;

};
}


#endif // PREDICTION_INTEGRAL_HPP
