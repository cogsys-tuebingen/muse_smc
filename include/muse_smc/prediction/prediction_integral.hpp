#ifndef PREDICTION_INTEGRAL_HPP
#define PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template<typename state_space_description_t, typename data_t>
class PredictionIntegral
{
public:
    using Ptr                = std::shared_ptr<PredictionIntegral>;
    using ConstPtr           = std::shared_ptr<PredictionIntegral const>;
    using prediction_model_t = PredictionModel<state_space_description_t, data_t>;

    PredictionIntegral() = default;
    virtual ~PredictionIntegral() = default;

    virtual void add(const typename prediction_model_t::Result::ConstPtr &step) = 0;
    virtual void reset() = 0;
    virtual bool updateThresholdExceeded() const = 0;
    virtual bool resamplingThresholdExceeded() const = 0;
    virtual bool isZero() const = 0;
    virtual void info() const = 0;
};
}

#endif // PREDICTION_INTEGRAL_HPP
