#ifndef DIFFERENTIAL_DRIVE_ABS_H
#define DIFFERENTIAL_DRIVE_ABS_H

#include <muse_mcl/plugins/types/prediction_model.hpp>
#include <muse_mcl/math/random.hpp>

namespace muse_mcl {
class DifferentialDriveAbs : public PredictionModel
{
public:
    DifferentialDriveAbs();

    virtual Result doPredict(const Data::ConstPtr &data,
                           const ros::Time &until,
                           ParticleSet::Poses set) override;

protected:
    unsigned int seed_;
    double       alpha_1_;
    double       alpha_2_;
    double       alpha_3_;
    double       alpha_4_;
    double       alpha_5_;

    math::random::Normal<1>::Ptr rng_delta_rot_hat1_;
    math::random::Normal<1>::Ptr rng_delta_trans_hat_;
    math::random::Normal<1>::Ptr rng_delta_rot_hat2_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // DIFFERENTIAL_DRIVE_ABS_H
