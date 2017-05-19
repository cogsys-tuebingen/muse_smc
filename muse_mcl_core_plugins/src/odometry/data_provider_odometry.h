#ifndef DATA_PROVIDER_ODOMETRY_H
#define DATA_PROVIDER_ODOMETRY_H

#include <nav_msgs/Odometry.h>

#include <muse_mcl/data_sources/data_provider.hpp>

namespace muse_mcl {
class DataProviderOdometry  : public muse_mcl::DataProvider
{
public:
    DataProviderOdometry() = default;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    nav_msgs::Odometry::ConstPtr last_msg_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
    void callback(const nav_msgs::OdometryConstPtr &msg);

};
}

#endif // DATA_PROVIDER_ODOMETRY_H
