#ifndef MOCK_UPDATE_H
#define MOCK_UPDATE_H

#include <muse_amcl/particle_filter/update_model.hpp>

namespace muse_amcl {
class MockUpdate : public UpdateModel
{
public:
    typedef std::shared_ptr<MockUpdate> Ptr;

    MockUpdate();

    virtual void update(const Data::ConstPtr &data,
                         const Map::ConstPtr &map,
                         ParticleSet::Weights set) override;

    double  first_parameter;
    int     second_parameter;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;


};
}

#endif /* MOCK_UPDATE_H */
