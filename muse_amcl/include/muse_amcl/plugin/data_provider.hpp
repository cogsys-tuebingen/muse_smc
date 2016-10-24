#pragma once

#include <memory>
#include <functional>
#include <ros/ros.h>
#include "data.hpp"

namespace muse_amcl {
class DataProvider {
public:
    typedef std::shared_ptr<DataProvider>       Ptr;
    typedef std::function<void(Data::ConstPtr)> Callback;

    struct Connection {

    };

    DataProvider() :
        nh_private("~"),
        connection_id(0)
    {
    }


    std::size_t connect(Callback &_slot)
    {

    }

    void disconnect()
    {

    }


private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    std::size_t     connection_id;
    std::map<std::size_t, Callback> connections;

};
}
