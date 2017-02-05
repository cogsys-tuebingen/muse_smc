#ifndef DIFFERENTIAL_DRIVE_H
#define DIFFERENTIAL_DRIVE_H

#include <muse_amcl/particle_filter/prediction_model.hpp>

namespace muse_amcl {
class DifferentialDrive : public PredictionModel
{
public:
    DifferentialDrive();

    virtual Result predict(const Data::ConstPtr &data,
                           const ros::Time &until,
                           ParticleSet::Poses set) override;

protected:
    double alpha_1_;
    double alpha_2_;
    double alpha_3_;
    double alpha_4_;
    double alpha_5_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif /* DIFFERENTIAL_DRIVE_H */
