#include "stereo_provider_2d.h"

#include <muse_mcl_2d_stereo/stereo_data.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_stereo::StereoProvider2D, muse_mcl_2d::DataProvider2D)

namespace muse_mcl_2d_stereo {
StereoProvider2D::StereoProvider2D() :
    time_offset_(0.0)
{
}

void StereoProvider2D::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if(!time_offset_.isZero() && !time_of_last_measurement_.isZero()) {
        if(msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;
    }

    muse_mcl_2d_stereo::StereoData::Ptr stereo_data(new muse_mcl_2d_stereo::StereoData(msg));
    data_received_(stereo_data);

    time_of_last_measurement_ = msg->header.stamp;
}

void StereoProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size = nh.param<int>(param_name("queue_size"), 1);
    topic_         = nh.param<std::string>(param_name("topic"), "");
    source_        = nh.subscribe(topic_, queue_size, &StereoProvider2D::callback, this);

    double rate    = nh.param<double>(param_name("rate"), 0.0);
    if(rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
        ROS_INFO_STREAM(name_ << ": Throttling stereo to rate of " << rate << "Hz!");
    }
}
}
