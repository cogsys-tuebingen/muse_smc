#include "data_provider_laser_2d.h"

#include <muse_amcl_core_plugins/laser_2d/laser_scan_2d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::DataProviderLaser2D, muse_amcl::DataProvider)

using namespace muse_amcl;

void DataProviderLaser2D::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/scan");
    source_= nh_private.subscribe(topic_, 1, &DataProviderLaser2D::callback, this);
    undistortion_ = nh_private.param<bool>(privateParameter("undistortion"), true);
    undistortion_fixed_frame_ = nh_private.param<std::string>(privateParameter("undistortion_fixed_frame"), "");
    undistortion_tf_timeout_ = ros::Duration(nh_private.param(privateParameter("undistotion_tf_timeout"), 0.1));

    range_max_ = nh_private.param<double>(privateParameter("range_max"), 30.0);
    range_min_ = nh_private.param<double>(privateParameter("range_min"), 0.05);
    angle_max_ = nh_private.param<double>(privateParameter("angle_max"), M_PI);
    angle_min_ = nh_private.param<double>(privateParameter("angle_min"),-M_PI);

}

void DataProviderLaser2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    LaserScan2D::Ptr laserscan(new LaserScan2D(msg->header.frame_id));

    auto convert = [&laserscan, &msg, this] () {
        const auto range_min       = msg->range_min;
        const auto range_max       = msg->range_max;
        const auto &ranges         = msg->ranges;
        const auto angle_increment = msg->angle_increment;
        auto angle                 = msg->angle_min;
        auto rays                  = laserscan->getRays();
        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    range >= range_min_ && range <= range_max_) {
                rays.emplace_back(LaserScan2D::Ray(angle, range));
            }
            angle += angle_increment;
        }
    };

    auto convertUndistorted = [&laserscan, &msg, this] () {
        const auto range_min       = msg->range_min;
        const auto range_max       = msg->range_max;
        const auto &ranges         = msg->ranges;
        const auto angle_increment = msg->angle_increment;
        const auto sensor_frame    = msg->header.frame_id;
        auto angle                 = msg->angle_min;
        auto rays                  = laserscan->getRays();

        const ros::Time end_time = msg->header.stamp;
        const ros::Time start_time = end_time - ros::Duration(msg->scan_time);
        TFProvider::LockedTFProvider tf_provider = tf_provider_->getLockedTFProvider();

        tf::StampedTransform fixed_T_end;
        if(!tf_provider.lookupTransform(undistortion_fixed_frame_,
                                        sensor_frame,
                                        end_time,
                                        fixed_T_end,
                                        undistortion_tf_timeout_)) {
            return false;
        }
        if(!tf_provider.waitForTransform(undistortion_fixed_frame_,
                                         sensor_frame,
                                         start_time,
                                         undistortion_tf_timeout_)) {
            return false;
        }

        if(ranges.size() == 0) {
            return false; /// this will avoid a division by zero and will end up in an empty scan message
        }

        ros::Time     stamp = start_time;
        ros::Duration stamp_delta = (end_time - start_time) * (1.0 / ranges.size());
        const tf::Transform end_T_fixed = fixed_T_end.inverse();

        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    range >= range_min_ && range <= range_max_) {
                tf::StampedTransform fixed_T_current;
                tf_provider.lookupTransform(undistortion_fixed_frame_,sensor_frame, stamp, fixed_T_current);

                tf::Transform end_T_current = end_T_fixed * fixed_T_current;
                math::Point pt = end_T_current * math::Point(std::cos(angle) * range, std::sin(angle) * range);
                rays.emplace_back(LaserScan2D::Ray(pt));
            }
            angle += angle_increment;
            stamp += stamp_delta;
        }
        return true;
    };

    if(undistortion_) {
        if(!convertUndistorted())
            convert();
    } else {
        convert();
    }

    data_received_(laserscan);
}
