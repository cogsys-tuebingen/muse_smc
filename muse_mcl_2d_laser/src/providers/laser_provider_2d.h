#ifndef LASER_PROVIDER_2D_H
#define LASER_PROVIDER_2D_H

#include <sensor_msgs/LaserScan.h>

#include <muse_mcl_2d/data/data_provider_2d.hpp>

namespace muse_mcl_2d_laser {
class LaserProvider2D : public muse_mcl_2d::DataProvider2D
{
public:
    using point_t = cslibs_math_2d::Point2d;
    using interval_t = std::array<double, 2>;

    LaserProvider2D();
    virtual ~LaserProvider2D() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to

    bool            undistortion_;              /// check if undistortion shall be applied
    std::string     undistortion_fixed_frame_;  /// the fixed frame necessary for the undistortion
    ros::Duration   undistortion_tf_timeout_;   /// time out for the tf listener

    interval_t linear_interval_;                /// linear field of view
    interval_t angular_interval_;               /// angular field of view

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::LaserScanConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // LASER_PROVIDER_2D_H
