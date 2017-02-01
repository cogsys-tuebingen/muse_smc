#ifndef DATA_PROVIDER_LASER_2D_H
#define DATA_PROVIDER_LASER_2D_H

#include <sensor_msgs/LaserScan.h>

#include <muse_amcl/data_sources/data_provider.hpp>

namespace muse_amcl {
class DataProviderLaser2D : public muse_amcl::DataProvider
{
public:
    DataProviderLaser2D() = default;

protected:
    virtual void doSetup(ros::NodeHandle &nh_private) override;

    ros::Subscriber source_;
    std::string     topic_;

    void callback(const sensor_msgs::LaserScanConstPtr &msg);


};
}

#endif // DATA_PROVIDER_LASER_2D_H
