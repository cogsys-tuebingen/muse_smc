#ifndef STEREO_PROVIDER_2D_H
#define STEREO_PROVIDER_2D_H

#include <sensor_msgs/PointCloud2.h>

#include <muse_mcl_2d/data/data_provider_2d.hpp>

namespace muse_mcl_2d_stereo {
class StereoProvider2D : public muse_mcl_2d::DataProvider2D
{
public:
    using point_t    = cslibs_math_2d::Point2d;
    using interval_t = std::array<double, 2>;

    StereoProvider2D();
    virtual ~StereoProvider2D() = default;

protected:
    ros::Subscriber source_;                    /// the subscriber to be used
    std::string     topic_;                     /// topic to listen to

    ros::Duration   time_offset_;
    ros::Time       time_of_last_measurement_;

    void callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    virtual void doSetup(ros::NodeHandle &nh) override;
};
}

#endif // STEREO_PROVIDER_2D_H
