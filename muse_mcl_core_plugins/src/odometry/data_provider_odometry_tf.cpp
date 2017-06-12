#include "data_provider_odometry_tf.h"

#include <muse_mcl_core_plugins/odometry/odometry.hpp>
#include <tf/tf.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DataProviderOdometryTF, muse_mcl::DataProvider)

using namespace muse_mcl;


DataProviderOdometryTF::DataProviderOdometryTF() :
    initialized_(false),
    rate_(60.0),
    running_(false),
    stop_(false)
{
}

DataProviderOdometryTF::~DataProviderOdometryTF()
{
    if(running_) {
        stop_ = true;
        if(worker_thread_.joinable())
            worker_thread_.join();
    }

}

void DataProviderOdometryTF::doSetup(ros::NodeHandle &nh_private)
{
    odom_frame_ = nh_private.param<std::string>(privateParameter("odom_frame"), "/odom");
    base_frame_ = nh_private.param<std::string>(privateParameter("base_frame"), "/base_link");
    rate_       = ros::Rate(nh_private.param<double>(privateParameter("rate"), 60.0));
    timeout_    = ros::Duration(nh_private.param<double>(privateParameter("timeout"), 0.1));

    if(!running_) {
        /// fire up the thread
        worker_thread_ = std::thread([this](){loop();});
        worker_thread_.detach();
    }
}

void DataProviderOdometryTF::loop()
{
    running_ = true;
    while(!stop_) {
        tf::StampedTransform o_T_b2;
        if(tf_.lookupTransform(odom_frame_, base_frame_, ros::Time::now(), o_T_b2, timeout_)) {
            if(initialized_) {
                TimeFrame time_frame(o_T_b1_.stamp_, o_T_b2.stamp_);
                Odometry::Ptr odometry(new Odometry(odom_frame_,
                                                    time_frame,
                                                    math::Pose(static_cast<tf::Transform>(o_T_b1_)),
                                                    math::Pose(static_cast<tf::Transform>(o_T_b2))));
                data_received_(odometry);
            } else {
                o_T_b1_ = o_T_b2;
            }
        }
        rate_.sleep();
    }
    running_ = false;
}
