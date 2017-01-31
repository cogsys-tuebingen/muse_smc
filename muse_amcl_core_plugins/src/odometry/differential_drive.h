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
    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* DIFFERENTIAL_DRIVE_H */
