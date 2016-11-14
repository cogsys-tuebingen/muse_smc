#pragma once

#include <muse_amcl/data_sources/map_provider.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace muse_amcl {
class MockMapProvider : public MapProvider
{
public:
    MockMapProvider();

    virtual Map::ConstPtr map() const override;

protected:
    virtual void loadParameters(ros::NodeHandle &nh_private) override;

};
}

