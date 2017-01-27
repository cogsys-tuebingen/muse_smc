#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include <muse_amcl/particle_filter/prediction_model.hpp>

namespace muse_amcl {
class OmniDrive : public PredictionModel
{
public:
    OmniDrive();

    virtual void predict(const Data::ConstPtr &data,
                         ParticleSet::Poses set) override;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif /* OMNI_DRIVE_H */
