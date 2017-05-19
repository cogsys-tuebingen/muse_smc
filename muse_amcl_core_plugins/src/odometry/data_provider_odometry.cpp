#include "data_provider_odometry.h"

#include <muse_amcl_core_plugins/odometry/odometry.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DataProviderOdometry, muse_mcl::DataProvider)

using namespace muse_mcl;

void DataProviderOdometry::callback(const nav_msgs::OdometryConstPtr &msg)
{
    auto toPose = [](const nav_msgs::OdometryConstPtr &msg)
    {
        tf::Pose p;
        tf::poseMsgToTF(msg->pose.pose, p);
        return math::Pose(p);
    };

    if(last_msg_) {
        TimeFrame time_frame(last_msg_->header.stamp, msg->header.stamp);
        Odometry::Ptr odometry(new Odometry(msg->header.frame_id, time_frame,
                                            toPose(last_msg_), toPose(msg)));
        data_received_(odometry);
    }
    last_msg_ = msg;
}

void DataProviderOdometry::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/odom");
    source_= nh_private.subscribe(topic_, 1, &DataProviderOdometry::callback, this);

    Logger &l = Logger::getLogger();
    l.info("topic_=" + topic_, "DataProvider:" + name_);
}
