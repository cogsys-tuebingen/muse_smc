#ifndef STEREO_DATA_HPP
#define STEREO_DATA_HPP

#include <muse_smc/data/data.hpp>

#include <cslibs_math_3d/linear/pointcloud.hpp>
#include <cslibs_math_ros/sensor_msgs/conversion_3d.hpp>

namespace muse_mcl_2d_stereo {
class StereoData : public muse_smc::Data
{
public:    
    using Ptr     = std::shared_ptr<StereoData>;
    using cloud_t = cslibs_math_3d::Pointcloud3d;

    StereoData(const sensor_msgs::PointCloud2ConstPtr &msg) :
        Data(msg->header.frame_id, cslibs_math_ros::sensor_msgs::conversion_3d::from(msg))
    {
        cslibs_math_ros::sensor_msgs::conversion_3d::from(msg, points_);
    }

    inline const typename cloud_t::Ptr& getPoints() const
    {
        return points_;
    }

private:
    typename cloud_t::Ptr points_;
};
}

#endif // STEREO_DATA_HPP
