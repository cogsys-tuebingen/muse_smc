#include "laser_provider_2d.h"

#include <muse_mcl_2d_laser/laserscan_2d.hpp>
#include <muse_mcl_2d_laser/convert.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_laser::LaserProvider2D, muse_mcl_2d::DataProvider2D)

namespace muse_mcl_2d_laser {
LaserProvider2D::LaserProvider2D() :
    time_offset_(0.0)
{
}

void LaserProvider2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if(!time_offset_.isZero() && !time_of_last_measurement_.isZero()) {
        if(msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;
    }

    muse_mcl_2d_laser::LaserScan2D::Ptr laserscan;
    if(undistortion_ &&
            !convertUndistorted(msg, linear_interval_, angular_interval_, tf_, undistortion_fixed_frame_, undistortion_tf_timeout_, laserscan)) {
        if(convert(msg, linear_interval_, angular_interval_, laserscan)) {
            data_received_(laserscan);
        }
    } else if(convert(msg, linear_interval_, angular_interval_, laserscan)) {
        data_received_(laserscan);
    }

    time_of_last_measurement_ = msg->header.stamp;
}


void LaserProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size              = nh.param<int>(param_name("queue_size"), 1);
    topic_                      = nh.param<std::string>(param_name("topic"), "/scan");
    source_                     = nh.subscribe(topic_, queue_size, &LaserProvider2D::callback, this);
    undistortion_               = nh.param<bool>(param_name("undistortion"), true);
    undistortion_fixed_frame_   = nh.param<std::string>(param_name("undistortion_fixed_frame"), "");
    undistortion_tf_timeout_    = ros::Duration(nh.param(param_name("undistotion_tf_timeout"), 0.1));

    linear_interval_[0]         = nh.param<double>(param_name("range_min"), 0.05);
    linear_interval_[1]         = nh.param<double>(param_name("range_max"), 30.0);
    angular_interval_[0]        = nh.param<double>(param_name("angle_min"),-M_PI);
    angular_interval_[1]        = nh.param<double>(param_name("angle_max"), M_PI);

    double rate                 = nh.param<double>(param_name("rate"), 0.0);
    if(rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling laser scan to rate of " << rate << "Hz!");
    }
}
}
