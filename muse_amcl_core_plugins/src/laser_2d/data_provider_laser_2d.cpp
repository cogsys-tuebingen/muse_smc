#include "data_provider_laser_2d.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>

using namespace muse_amcl;

void DataProviderLaser2D::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/scan");
    source_= nh_private.subscribe(topic_, 1, &DataProviderLaser2D::callback, this);
}

void DataProviderLaser2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    /// do the undistortion using tf
}
