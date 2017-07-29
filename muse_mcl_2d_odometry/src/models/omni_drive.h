#ifndef OMNI_DRIVE_H
#define OMNI_DRIVE_H

#include <muse_mcl/prediction/model_prediction.hpp>

namespace muse_mcl {
class OmniDrive : public ModelPrediction
{
public:
    OmniDrive();

    virtual Result doPredict(const Data::ConstPtr &data,
                           const ros::Time      &until,
                           ParticleSet::Poses set) override;

protected:
    unsigned int seed_;
    double alpha_1_;
    double alpha_2_;
    double alpha_3_;
    double alpha_4_;
    double alpha_5_;

    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif /* OMNI_DRIVE_H */
