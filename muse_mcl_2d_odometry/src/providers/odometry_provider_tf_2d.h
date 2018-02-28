#ifndef DATA_PROVIDER_ODOMETRY_TF_H
#define DATA_PROVIDER_ODOMETRY_TF_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <muse_mcl_2d/data/data_provider_2d.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

namespace muse_mcl_2d_odometry {
class OdometryProviderTF2D : public muse_mcl_2d::DataProvider2D
{
public:
    using stamped_t = cslibs_time::Stamped<cslibs_math_2d::Transform2d>;

    OdometryProviderTF2D();
    virtual ~OdometryProviderTF2D();

protected:
    cslibs_math_ros::tf::TFListener2d     tf_;
    std::string                           odom_frame_;
    std::string                           base_frame_;

    stamped_t                             o_T_b1_;
    bool                                  initialized_;
    ros::Rate                             rate_;
    ros::Duration                         timeout_;
    std::atomic_bool                      running_;
    std::atomic_bool                      stop_;
    std::thread                           worker_thread_;

    virtual void doSetup(ros::NodeHandle &nh) override;
    void loop();
};
}

#endif // DATA_PROVIDER_ODOMETRY_TF_H
