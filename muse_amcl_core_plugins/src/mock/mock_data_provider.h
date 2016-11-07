#pragma once

#include <muse_amcl/plugins/types/data_provider.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace muse_amcl {
class MockDataProvider : public DataProvider
{
public:
    typedef std::shared_ptr<MockDataProvider> Ptr;

    MockDataProvider();

protected:
    virtual void loadParameters(ros::NodeHandle &nh_private) override;

    ros::Subscriber source;
    std::string     topic;

    void callback(const std_msgs::String::ConstPtr &msg);

};
}
