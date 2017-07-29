#ifndef DATA_PROVIDER_ODOMETRY_TF_H
#define DATA_PROVIDER_ODOMETRY_TF_H

#include <ros/ros.h>
#include <thread>
#include <atomic>

#include <muse_mcl/providers/provider_data.hpp>
#include <muse_mcl/tf/tf_provider.hpp>

namespace muse_mcl {
class ProviderDataOdometryTF : public muse_mcl::ProviderData
{
public:
    ProviderDataOdometryTF();

    virtual ~ProviderDataOdometryTF();

protected:
    TFProvider           tf_;
    std::string          odom_frame_;
    std::string          base_frame_;

    tf::StampedTransform o_T_b1_;
    bool                 initialized_;
    ros::Rate            rate_;
    ros::Duration        timeout_;

    virtual void doSetup(ros::NodeHandle &nh_private) override;
    void loop();
    std::atomic_bool running_;
    std::atomic_bool stop_;
    std::thread      worker_thread_;

};
}

#endif // DATA_PROVIDER_ODOMETRY_TF_H
