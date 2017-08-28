#include "odometry_provider_2d.h"

#include <muse_mcl_2d/odometry/odometry_2d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_odometry::OdometryProvider2D, muse_mcl_2d::DataProvider2D)

using namespace muse_mcl_2d_odometry;
using namespace muse_mcl_2d;

void OdometryProvider2D::callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto to_pose = [](const nav_msgs::OdometryConstPtr &msg)
    {
        return Pose2D(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      tf::getYaw(msg->pose.pose.orientation));
    };

    if(last_msg_) {
        muse_smc::TimeFrame time_frame(last_msg_->header.stamp.toNSec(),
                                       msg->header.stamp.toNSec());
        Odometry2D::Ptr odometry(new Odometry2D(msg->header.frame_id,
                                                time_frame,
                                                to_pose(last_msg_),
                                                to_pose(msg)));

        data_received_(odometry);
    }
    last_msg_ = msg;
}

void OdometryProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    const int queue_size = nh.param<int>(param_name("queue_size"), 1);
    topic_ = nh.param<std::string>(param_name("topic"), "/odom");
    source_= nh.subscribe(topic_, queue_size, &OdometryProvider2D::callback, this);
}
