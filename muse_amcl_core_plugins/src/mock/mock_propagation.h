#ifndef MOCK_PROPAGATION_H
#define MOCK_PROPAGATION_H


#include <muse_amcl/particle_filter/propagation.hpp>

namespace muse_amcl {
class MockPropagation : public Propagation
{
public:
    typedef std::shared_ptr<MockPropagation> Ptr;

    MockPropagation();

    virtual void apply(const Data::ConstPtr &data,
                       ParticleSet::PoseIterator set) override;


    double  first_parameter;
    int     second_parameter;

protected:
    virtual void doSetup(ros::NodeHandle &nh) override;


};
}

#endif /* MOCK_PROPAGATION_H */