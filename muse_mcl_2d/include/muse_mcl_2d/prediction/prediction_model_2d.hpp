#ifndef PREDICTION_MODEL_2D_HPP
#define PREDICTION_MODEL_2D_HPP

#include <muse_smc/prediction/prediction_model.hpp>

#include <muse_mcl_2d/samples/sample_2d.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>

namespace muse_mcl_2d {
class PredictionModel2D : public muse_smc::PredictionModel<Sample2D>
{
public:
    PredictionModel2D() = default;
    virtual ~PredictionModel2D() = default;

    inline void setup(const TFProvider::Ptr &tf,
                      ros::NodeHandle &nh)
    {
        auto param_name = [this](const std::string &name){return name_ + "/" + name;};
        tf_ = tf;
        eps_zero_linear_  = nh.param(param_name("eps_zero_linear"),  1e-4);
        eps_zero_angular_ = nh.param(param_name("eps_zero_angular"), 1e-4);
        doSetup(nh);
    }

protected:
    TFProvider::Ptr tf_;
    double          eps_zero_linear_;
    double          eps_zero_angular_;

    virtual void doSetup(ros::NodeHandle &nh) = 0;

};
}

#endif // PREDICTION_MODEL_2D_HPP
