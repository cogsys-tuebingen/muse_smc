#ifndef MOCK_PROPAGATION_H
#define MOCK_PROPAGATION_H


#include <muse_amcl/particle_filter/prediction_model.hpp>

namespace muse_amcl {
class MockPropagation : public PredictionModel
{
public:
    typedef std::shared_ptr<MockPropagation> Ptr;

    MockPropagation();

    virtual bool predict(const Data::ConstPtr &data,
                         ParticleSet::Poses set) override;


    double  first_parameter;
    int     second_parameter;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;


};
}

#endif /* MOCK_PROPAGATION_H */
