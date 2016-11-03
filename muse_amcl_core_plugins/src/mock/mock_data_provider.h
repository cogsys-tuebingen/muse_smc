#pragma once

#include <muse_amcl/plugins/data_provider.hpp>

namespace muse_amcl {
class MockDataProvider : public DataProvider
{
public:
    typedef std::shared_ptr<MockDataProvider> Ptr;

    MockDataProvider();

protected:
    virtual void loadParameters(ros::NodeHandle &nh_private) override;


};
}
