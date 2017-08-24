#ifndef DATA_PROVIDER_ODOMETRY_H
#define DATA_PROVIDER_ODOMETRY_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <muse_mcl_2d/data/data_provider_2d.hpp>

namespace muse_mcl_2d_odometry {
class OdometryProvider2D : public muse_mcl_2d::DataProvider2D
{
public:
    OdometryProvider2D() = default;

protected:
    ros::Subscriber source_;
    std::string     topic_;

    nav_msgs::Odometry::ConstPtr last_msg_;

    void callback(const nav_msgs::OdometryConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;

};
}

#endif // DATA_PROVIDER_ODOMETRY_H
