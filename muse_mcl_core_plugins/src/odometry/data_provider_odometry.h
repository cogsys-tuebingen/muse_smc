#ifndef DATA_PROVIDER_ODOMETRY_H
#define DATA_PROVIDER_ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <muse_mcl/plugins/types/provider_data.hpp>

namespace muse_mcl {
class ProviderDataOdometry  : public muse_mcl::ProviderData
{
public:
    ProviderDataOdometry() = default;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    nav_msgs::Odometry::ConstPtr last_msg_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
    void callback(const nav_msgs::OdometryConstPtr &msg);

};
}

#endif // DATA_PROVIDER_ODOMETRY_H
