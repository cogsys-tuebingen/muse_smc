#include "odometry_provider_tf_2d.h"

#include <muse_mcl_2d/odometry/odometry_2d.hpp>
#include <tf/tf.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_odometry::OdometryProviderTF2D, muse_mcl_2d::DataProvider2D)

using namespace muse_mcl_2d_odometry;
using namespace muse_mcl_2d;

OdometryProviderTF2D::OdometryProviderTF2D() :
    o_T_b1_(math::Transform2D(), muse_smc::Time(ros::Time::now().toNSec())),
    initialized_(false),
    rate_(60.0),
    running_(false),
    stop_(false)
{
}

OdometryProviderTF2D::~OdometryProviderTF2D()
{
    if(running_) {
        stop_ = true;
        if(worker_thread_.joinable())
            worker_thread_.join();
    }
}

void OdometryProviderTF2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    odom_frame_ = nh.param<std::string>(param_name("odom_frame"), "/odom");
    base_frame_ = nh.param<std::string>(param_name("base_frame"), "/base_link");
    rate_       = ros::Rate(nh.param<double>(param_name("rate"), 70.0));
    timeout_    = ros::Duration(nh.param<double>(param_name("timeout"), 0.1));

    if(!running_) {
        /// fire up the thread
        worker_thread_ = std::thread([this](){loop();});
        worker_thread_.detach();
    }
}

void OdometryProviderTF2D::loop()
{
    running_ = true;
    while(!stop_) {
        const ros::Time now = ros::Time::now();
        muse_mcl_2d::math::StampedTransform2D o_T_b2(math::Transform2D(), muse_smc::Time(now.toNSec()));
        if(tf_.lookupTransform(odom_frame_, base_frame_, now, o_T_b2, timeout_)) {
            if(initialized_) {
                muse_smc::TimeFrame time_frame(o_T_b1_.stamp(), o_T_b2.stamp());
                Odometry2D::Ptr odometry(new Odometry2D(odom_frame_,
                                                        time_frame,
                                                        o_T_b1_.data(),
                                                        o_T_b2.data()));
                data_received_(odometry);
            } else {
                initialized_ = true;
            }
            o_T_b1_ = o_T_b2;
        }
        rate_.sleep();
    }
    running_ = false;
}
