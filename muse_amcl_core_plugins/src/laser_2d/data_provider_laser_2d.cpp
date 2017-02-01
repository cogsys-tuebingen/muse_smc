#include "data_provider_laser_2d.h"

#include "laser_scan_2d.hpp"

using namespace muse_amcl;

void DataProviderLaser2D::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/scan");
    source_= nh_private.subscribe(topic, 1, &DataProviderLaser2D::callback, this);
}

void DataProviderLaser2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    //// setup the sensor data


}
