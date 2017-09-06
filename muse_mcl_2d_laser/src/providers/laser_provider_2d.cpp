#include "laser_provider_2d.h"

#include <muse_mcl_2d_laser/laser/laser_2d_scan.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_laser::LaserProvider2D, muse_mcl_2d::DataProvider2D)

using namespace muse_mcl_2d_laser;
using namespace muse_mcl_2d;

LaserProvider2D::LaserProvider2D() :
    time_offset_(0.0)
{
}

void LaserProvider2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
{
    if(!time_offset_.isZero() &&
            time_of_last_measurement_.isZero()) {
        if(msg->header.stamp <= (time_of_last_measurement_ + time_offset_))
            return;
    }

    const ros::Time start_stamp   = msg->header.stamp;
    ros::Duration delta_stamp     = ros::Duration(msg->time_increment);
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(msg->scan_time / static_cast<double>(msg->ranges.size()));
    }
    const ros::Time end_stamp = msg->header.stamp + delta_stamp * msg->ranges.size();

    LaserScan2D::time_frame_t time_frame(start_stamp.toNSec(), end_stamp.toNSec());
    LaserScan2D::Ptr laserscan(new LaserScan2D(msg->header.frame_id,
                                               time_frame));

    auto convert = [&laserscan, &msg, this] () {
        const auto range_min       = msg->range_min;
        const auto range_max       = msg->range_max;
        const auto &ranges         = msg->ranges;
        const auto angle_increment = msg->angle_increment;
        auto angle                 = msg->angle_min;
        auto &scan                 = *laserscan;
        scan.setAngleInterval(std::max(angle_min_, (double) msg->angle_min),
                              std::min(angle_max_, (double) msg->angle_max));
        scan.setRangeInterval(std::max(range_min_, (double) range_min),
                              std::min(range_max_, (double) range_max));
        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    angle >= angle_min_ && angle <= angle_max_) {
                scan.insert(angle, range);
            } else {
                scan.insertInvalid();
            }
            angle += angle_increment;
        }
    };

    auto convertUndistorted = [&laserscan, &msg, &start_stamp, &delta_stamp, &end_stamp, this] () {
        const auto range_min       = msg->range_min;
        const auto range_max       = msg->range_max;
        const auto &ranges         = msg->ranges;
        const auto angle_increment = msg->angle_increment;
        const auto sensor_frame    = msg->header.frame_id;
        auto angle                 = msg->angle_min;
        auto &scan                 = *laserscan;

        tf::StampedTransform fixed_T_end;
        if(!tf_->lookupTransform(undistortion_fixed_frame_,
                                          sensor_frame,
                                          end_stamp,
                                          fixed_T_end,
                                          undistortion_tf_timeout_)) {
            return false;
        }
        if(!tf_->waitForTransform(undistortion_fixed_frame_,
                                           sensor_frame,
                                           start_stamp,
                                           undistortion_tf_timeout_)) {
            return false;
        }

        if(ranges.size() == 0) {
            return false; /// this will avoid a division by zero and will end up in an empty scan message
        }

        ros::Time           stamp       = start_stamp;
        const tf::Transform end_T_fixed = fixed_T_end.inverse();

        scan.setAngleInterval(std::max(angle_min_, (double) msg->angle_min),
                              std::min(angle_max_,(double) msg->angle_max));
        scan.setRangeInterval(std::max(range_min_,(double) range_min),
                              std::min(range_max_,(double) range_max));

        for(const auto range : ranges) {
            if(range >= range_min && range <= range_max &&
                    angle >= angle_min_ && angle <= angle_max_) {
                tf::StampedTransform fixed_T_current;
                tf_->lookupTransform(undistortion_fixed_frame_,sensor_frame, stamp, fixed_T_current);

                tf::Transform end_T_current = end_T_fixed * fixed_T_current;
                tf::Point pt = end_T_current * tf::Point(std::cos(angle) * range, std::sin(angle) * range, 0.0);
                scan.insert(point_t(pt.x(), pt.y()));
            } else {
                scan.insertInvalid();
            }
            angle += angle_increment;
            stamp += delta_stamp;
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


void LaserProvider2D::doSetup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    int queue_size = nh.param<int>(param_name("queue_size"), 1);

    topic_ = nh.param<std::string>(param_name("topic"), "/scan");
    source_= nh.subscribe(topic_, queue_size, &LaserProvider2D::callback, this);
    undistortion_ = nh.param<bool>(param_name("undistortion"), true);
    undistortion_fixed_frame_ = nh.param<std::string>(param_name("undistortion_fixed_frame"), "");
    undistortion_tf_timeout_ = ros::Duration(nh.param(param_name("undistotion_tf_timeout"), 0.1));

    range_max_ = nh.param<double>(param_name("range_max"), 30.0);
    range_min_ = nh.param<double>(param_name("range_min"), 0.05);
    angle_max_ = nh.param<double>(param_name("angle_max"), M_PI);
    angle_min_ = nh.param<double>(param_name("angle_min"),-M_PI);

    double rate = nh.param<double>(param_name("rate"), 0.0);
    if(rate > 0.0) {
        time_offset_ = ros::Duration(1.0 / rate);
    }
}
