#include "mock_data_provider.h"

#include "mock_data.hpp"

#include <class_loader/class_loader_register_macro.h>


CLASS_LOADER_REGISTER_CLASS(muse_amcl::MockDataProvider, muse_amcl::DataProvider)

using namespace muse_amcl;

MockDataProvider::MockDataProvider()
{

}

void MockDataProvider::doSetup(ros::NodeHandle &nh_private)
{
    ros::NodeHandle nh;
    topic = nh_private.param<std::string>(privateParameter("topic"), "/mock");

    std::cout << "My name is MockDataProvider and I listen to topic: " << topic << std::endl;;

    source = nh.subscribe(topic, 1, &MockDataProvider::callback, this);
}

void MockDataProvider::callback(const std_msgs::String::ConstPtr& msg)
{
    MockData *d = new MockData;
    d->value = msg->data;

    Data::ConstPtr ptr(d);

    data_received_(ptr);
}
