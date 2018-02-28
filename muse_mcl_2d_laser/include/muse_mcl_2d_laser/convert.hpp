#ifndef CONVERT_HPP
#define CONVERT_HPP

#include <sensor_msgs/LaserScan.h>

#include <muse_mcl_2d_laser/laserscan_2d.hpp>
#include <cslibs_math_ros/tf/tf_listener_2d.hpp>

namespace muse_mcl_2d_laser {

using interval_t = std::array<double, 2>;

inline LaserScan2D::Ptr create(const sensor_msgs::LaserScanConstPtr &src,
                               const interval_t                     &linear_interval,
                               const interval_t                     &angular_interval)
{
    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(src->time_increment) * static_cast<double>(src->ranges.size());
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(src->scan_time);
    }

    const double fov             = src->angle_max - src->angle_min;
    const double start_off_ratio = (std::max(static_cast<double>(src->angle_min), angular_interval[0]) - src->angle_min) / fov;
    const double end_off_ratio   = (std::min(static_cast<double>(src->angle_max), angular_interval[1]) - src->angle_min) / fov;

    const int64_t start_time = static_cast<int64_t>(std::floor(start_stamp.toNSec() + delta_stamp.toNSec() * start_off_ratio + 0.5));
    const int64_t end_time   = static_cast<int64_t>(std::floor(start_stamp.toNSec() + delta_stamp.toNSec() * end_off_ratio  + 0.5));

    const  LaserScan2D::time_frame_t time_frame(start_time, end_time);
    return LaserScan2D::Ptr (new LaserScan2D(src->header.frame_id, time_frame, linear_interval, angular_interval, cslibs_time::Time(ros::Time::now().toNSec())));
}


inline bool convert(const sensor_msgs::LaserScanConstPtr &src,
                    const interval_t                     &linear_interval,
                    const interval_t                     &angular_interval,
                    LaserScan2D::Ptr                     &dst)
{
    const auto src_linear_min  = static_cast<double>(src->range_min);
    const auto src_linear_max  = static_cast<double>(src->range_max);
    const auto src_angular_min = static_cast<double>(src->angle_min);
    const auto src_angular_max = static_cast<double>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if(src_ranges.size() == 0ul)
        return false;

    const interval_t dst_linear_interval  = {std::max(linear_interval[0],  src_linear_min),
                                             std::min(linear_interval[1],  src_linear_max)};
    const interval_t dst_angular_interval = {std::max(angular_interval[0], src_angular_min),
                                             std::min(angular_interval[1], src_angular_max)};

    dst = create(src, dst_linear_interval, dst_angular_interval);

    auto in_linear_interval = [&dst_linear_interval](const double range)
    {
        return range >= dst_linear_interval[0] && range <= dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const double angle)
    {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };

    auto angle = src_angular_min;
    for(const auto range : src_ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle)) {
            dst->insert(angle, range);
        } else {
            dst->insertInvalid();
        }
        angle += src_angle_increment;
    }
    return true;
}

inline bool convertUndistorted(const sensor_msgs::LaserScanConstPtr     &src,
                               const interval_t                         &linear_interval,
                               const interval_t                         &angular_interval,
                               cslibs_math_ros::tf::TFListener2d::Ptr   &tf_listener,
                               const std::string                        &fixed_frame,
                               const ros::Duration                      &tf_timeout,
                               LaserScan2D::Ptr                         &dst)
{

    const auto src_linear_min  = static_cast<double>(src->range_min);
    const auto src_linear_max  = static_cast<double>(src->range_max);
    const auto src_angular_min = static_cast<double>(src->angle_min);
    const auto src_angular_max = static_cast<double>(src->angle_max);
    const auto &src_ranges         = src->ranges;
    const auto src_angle_increment = src->angle_increment;

    if(src_ranges.size() == 0ul)
        return false;

    const interval_t dst_linear_interval  = {{std::max(linear_interval[0],  src_linear_min),
                                             std::min(linear_interval[1],  src_linear_max)}};
    const interval_t dst_angular_interval = {{std::max(angular_interval[0], src_angular_min),
                                             std::min(angular_interval[1], src_angular_max)}};

    dst = create(src, dst_linear_interval, dst_angular_interval);

    auto in_linear_interval = [&dst_linear_interval](const double range)
    {
        return range >= dst_linear_interval[0] && range <= dst_linear_interval[1];
    };
    auto in_angular_interval = [&dst_angular_interval](const double angle)
    {
        return angle >= dst_angular_interval[0] && angle <= dst_angular_interval[1];
    };


    const ros::Time start_stamp = src->header.stamp;
    ros::Duration   delta_stamp = ros::Duration(src->time_increment);
    if(delta_stamp <= ros::Duration(0.0)) {
        delta_stamp = ros::Duration(src->scan_time / static_cast<float>(src_ranges.size()));
    }
    const ros::Time end_stamp = start_stamp + delta_stamp * src_ranges.size();

    tf::StampedTransform start_T_end;
    if(!tf_listener->lookupTransform(fixed_frame, dst->getFrame(), end_stamp, start_T_end, tf_timeout)) {
        return false;
    }
    if(!tf_listener->waitForTransform(fixed_frame, dst->getFrame(), start_stamp, tf_timeout)) {
        return false;
    }

    tf::Transform end_T_start = start_T_end.inverse();

    auto angle = src_angular_min;
    auto stamp = start_stamp;
    for(const auto range : src_ranges) {
        if(in_linear_interval(range) && in_angular_interval(angle)) {
            tf::StampedTransform start_T_stamp;
            tf_listener->lookupTransform(fixed_frame, dst->getFrame(), stamp, start_T_stamp);
            tf::Transform end_T_stamp = end_T_start * start_T_stamp;
            tf::Point pt = end_T_stamp * tf::Point(std::cos(angle) * range, std::sin(angle) * range, 0.0);
            dst->insert(cslibs_math_2d::Point2d(pt.x(), pt.y()));
        } else {
            dst->insertInvalid();
        }
        angle       += src_angle_increment;
        stamp += delta_stamp;
    }
    return true;
}
}


#endif // CONVERT_HPP
