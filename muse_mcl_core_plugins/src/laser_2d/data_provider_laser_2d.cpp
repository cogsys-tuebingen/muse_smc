#include "data_provider_laser_2d.h"

#include <muse_mcl_core_plugins/laser_2d/laser_scan_2d.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::DataProviderLaser2D, muse_mcl::DataProvider)

using namespace muse_mcl;

void DataProviderLaser2D::doSetup(ros::NodeHandle &nh_private)
{
    int queue_size = nh_private.param<int>(privateParameter("queue_size"), 1);

    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/scan");
    source_= nh_private.subscribe(topic_, queue_size, &DataProviderLaser2D::callback, this);
    undistortion_ = nh_private.param<bool>(privateParameter("undistortion"), true);
    undistortion_fixed_frame_ = nh_private.param<std::string>(privateParameter("undistortion_fixed_frame"), "");
    undistortion_tf_timeout_ = ros::Duration(nh_private.param(privateParameter("undistotion_tf_timeout"), 0.1));

    range_max_ = nh_private.param<double>(privateParameter("range_max"), 30.0);
    range_min_ = nh_private.param<double>(privateParameter("range_min"), 0.05);
    angle_max_ = nh_private.param<double>(privateParameter("angle_max"), M_PI);
    angle_min_ = nh_private.param<double>(privateParameter("angle_min"),-M_PI);

    time_offset_ = ros::Rate(nh_private.param<double>(privateParameter("rate"), 0.0)).cycleTime();

    Logger &l = Logger::getLogger();
    l.info("topic_='" + topic_ + "'", "DataProvider:" + name_);
    l.info("undistortion_='" + std::to_string(undistortion_) + "'", "DataProvider:" + name_);
    l.info("undistortion_fixed_frame_='" + undistortion_fixed_frame_ + "'", "DataProvider:" + name_);
    l.info("undistortion_tf_timeout_='" + std::to_string(undistortion_tf_timeout_.toSec()) + "'", "DataProvider:" + name_);

    l.info("range_max_='" + std::to_string(range_max_) + "'", "DataProvider:" + name_);
    l.info("range_min_='" + std::to_string(range_min_) + "'", "DataProvider:" + name_);
    l.info("angle_max_='" + std::to_string(angle_max_) + "'", "DataProvider:" + name_);
    l.info("angle_min_='" + std::to_string(angle_min_) + "'", "DataProvider:" + name_);

}

void DataProviderLaser2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if(!time_offset_.isZero() &&
            time_of_last_measurement_.isZero()) {
        if(msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;
    }

    LaserScan2D::Ptr laserscan(new LaserScan2D(msg->header.frame_id));

    auto convert = [&laserscan, &msg, this] () {
        const auto range_min       = msg->range_min;
        const auto range_max       = msg->range_max;
        const auto &ranges         = msg->ranges;
        const auto angle_increment = msg->angle_increment;
        auto angle                 = msg->angle_min;
        auto &rays                  = laserscan->getRays();
        laserscan->setAngleInterval(std::max(angle_min_, (double) msg->angle_min),
                                    std::min(angle_max_, (double) msg->angle_max));
        laserscan->setRangeInterval(std::max(range_min_, (double) range_min),
                                    std::min(range_max_, (double) range_max));
        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    angle >= angle_min_ && angle <= angle_max_) {
                rays.emplace_back(LaserScan2D::Ray(angle, range));
            } else {
                rays.emplace_back(LaserScan2D::Ray());
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
        auto &rays                  = laserscan->getRays();

        const ros::Time end_time = msg->header.stamp;
        const ros::Time start_time = end_time - ros::Duration(msg->scan_time);

        tf::StampedTransform fixed_T_end;
        if(!tf_provider_->lookupTransform(undistortion_fixed_frame_,
                                          sensor_frame,
                                          end_time,
                                          fixed_T_end,
                                          undistortion_tf_timeout_)) {
            return false;
        }
        if(!tf_provider_->waitForTransform(undistortion_fixed_frame_,
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

        laserscan->setAngleInterval(std::max(angle_min_, (double) msg->angle_min),
                                    std::min(angle_max_,(double) msg->angle_max));
        laserscan->setRangeInterval(std::max(range_min_,(double) range_min),
                                    std::min(range_max_,(double) range_max));

        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    angle >= angle_min_ && angle <= angle_max_) {
                tf::StampedTransform fixed_T_current;
                tf_provider_->lookupTransform(undistortion_fixed_frame_,sensor_frame, stamp, fixed_T_current);

                tf::Transform end_T_current = end_T_fixed * fixed_T_current;
                math::Point pt = end_T_current * math::Point(std::cos(angle) * range, std::sin(angle) * range);
                rays.emplace_back(LaserScan2D::Ray(pt));
            } else {
                rays.emplace_back(LaserScan2D::Ray());
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
    time_of_last_measurement_ = msg->header.stamp;
}
