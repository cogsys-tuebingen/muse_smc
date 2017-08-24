#ifndef DATA_PROVIDER_ODOMETRY_TF_H
#define DATA_PROVIDER_ODOMETRY_TF_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <muse_mcl_2d/data/data_provider_2d.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>

namespace muse_mcl_2d_odometry {
class OdometryProviderTF2D : public muse_mcl_2d::DataProvider2D
{
public:
    OdometryProviderTF2D();
    virtual ~OdometryProviderTF2D();

protected:
    muse_mcl_2d::TFProvider         tf_;
    std::string                     odom_frame_;
    std::string                     base_frame_;

    muse_mcl_2d::StampedTransform2D o_T_b1_;
    bool                            initialized_;
    ros::Rate                       rate_;
    ros::Duration                   timeout_;
    std::atomic_bool                running_;
    std::atomic_bool                stop_;
    std::thread                     worker_thread_;

    virtual void doSetup(ros::NodeHandle &nh) override;
    void loop();
};
}

#endif // DATA_PROVIDER_ODOMETRY_TF_H
