#ifndef MOCK_MAP_PROVIDER_H
#define MOCK_MAP_PROVIDER_H

#include <muse_amcl/data_sources/map_provider.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>

namespace muse_amcl {
class MockMapProvider : public MapProvider
{
public:
    MockMapProvider();

    virtual Map::ConstPtr getMap() const override;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;

};
}

#endif /* MOCK_MAP_PROVIDER_H */
