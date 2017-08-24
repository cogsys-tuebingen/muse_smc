#include "provider_data_laser_2d.h"

#include <muse_mcl_2d_laser/laser/laser_2d_scan.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_laser::LaserProvider2D, muse_mcl_2d::DataProvider2D)

using namespace muse_mcl_2d_laser;
using namespace muse_mcl_2d;

void LaserProvider2D::callback(const sensor_msgs::LaserScanConstPtr &msg)
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
        if(!tf_->lookupTransform(undistortion_fixed_frame_,
                                          sensor_frame,
                                          end_time,
                                          fixed_T_end,
                                          undistortion_tf_timeout_)) {
            return false;
        }
        if(!tf_->waitForTransform(undistortion_fixed_frame_,
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
                tf_->lookupTransform(undistortion_fixed_frame_,sensor_frame, stamp, fixed_T_current);

                tf::Transform end_T_current = end_T_fixed * fixed_T_current;
                tf::Point pt = end_T_current * tf::Point(std::cos(angle) * range, std::sin(angle) * range, 0.0);
                rays.emplace_back(LaserScan2D::Ray(point_t(pt.x(), pt.y())));
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

    time_offset_ = ros::Rate(nh.param<double>(param_name("rate"), 0.0)).cycleTime();
}
