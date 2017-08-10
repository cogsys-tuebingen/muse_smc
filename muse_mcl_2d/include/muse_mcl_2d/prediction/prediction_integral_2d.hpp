#ifndef PREDICTION_INTEGRAL_2D_HPP
#define PREDICTION_INTEGRAL_2D_HPP

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_smc/prediction/prediction_integral.hpp>

namespace muse_mcl_2d {
class PredictionIntegral2D : public muse_smc::PredictionIntegral<Sample2D>
{
public:
    PredictionIntegral2D() :
        linear_distance_abs_(0.0),
        angular_distance_abs_(0.0)
    {
    }

    virtual ~PredictionIntegral2D() = default;

    virtual void add(typename prediction_model_t::Result::Ptr &step) = 0;

    virtual void reset() override
    {
        linear_distance_abs_ = 0.0;
        angular_distance_abs_ = 0.0;
    }

    virtual bool isZero() const override
    {
        return linear_distance_abs_ != 0.0 ||
               angular_distance_abs_ != 0.0;
    }

private:
    double linear_distance_abs_;
    double angular_distance_abs_;
};
}


#endif // PREDICTION_INTEGRAL_2D_HPP

