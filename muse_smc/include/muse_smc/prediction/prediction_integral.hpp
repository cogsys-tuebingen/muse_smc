#ifndef PREDICTION_INTEGRAL_HPP
#define PREDICTION_INTEGRAL_HPP

#include <muse_smc/prediction/prediction_model.hpp>

namespace muse_smc {
template<typename sample_t>
class PredictionIntegral
{
public:
    using Ptr = std::shared_ptr<PredictionIntegral<sample_t>>;
    using ConstPtr = std::shared_ptr<PredictionIntegral<sample_t> const>;
    using prediction_model_t = PredictionModel<sample_t>;

    PredictionIntegral() = default;
    virtual ~PredictionIntegral() = default;

    virtual void add(typename prediction_model_t::Result::Ptr &step) = 0;

    virtual void reset() = 0;
    virtual bool isZero() const = 0;

    /**
      * define minus
      * define if predictions were applied
      */

};
}


#endif // PREDICTION_INTEGRAL_HPP
