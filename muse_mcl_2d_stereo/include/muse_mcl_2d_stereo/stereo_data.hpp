#ifndef STEREO_DATA_HPP
#define STEREO_DATA_HPP

#include <muse_smc/data/data.hpp>

#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>

#include <limits>
#include <vector>

namespace muse_mcl_2d_stereo {
class StereoData : public muse_smc::Data
{
public:    
    using Ptr = std::shared_ptr<StereoData>;

    StereoData(const sensor_msgs::PointCloud2ConstPtr &msg) :
        Data(msg->header.frame_id, cslibs_math_ros::sensor_msgs::conversion_3d::from(msg))
    {
        cslibs_math_ros::sensor_msgs::conversion_3d::from(msg, points_);
    }

private:
    cslibs_math_3d::Pointcloud3d::Ptr points_;
};
}

#endif // STEREO_DATA_HPP
