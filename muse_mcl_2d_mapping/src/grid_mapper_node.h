#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "occupancy_grid_mapper_2d.h"
#include "ndt_grid_mapper_2d.h"
#include "ndt_grid_mapper_3d.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <muse_mcl_2d/tf/tf_provider.hpp>
#include <muse_mcl_2d_laser/convert.hpp>
#include <cslibs_math_3d/conversion/tf.hpp>

namespace muse_mcl_mapping {
class GridMapperNode
{    
private:
    using interval_t   = std::array<double, 2>;
    using occ_map_2d_t = muse_mcl_2d_mapping::OccupancyGridMapper2d;
    using ndt_map_2d_t = muse_mcl_2d_mapping::NDTGridMapper2d;
    using ndt_map_3d_t = muse_mcl_3d_mapping::NDTGridMapper3d;
    using msg_2d_t     = nav_msgs::OccupancyGrid;
    using msg_3d_t     = sensor_msgs::PointCloud2;

    template <typename map_t, typename msg_t>
    struct MapperWorker {
        typename map_t::Ptr mapper_;
        std::string         map_frame_;
        ros::Publisher      pub_map_;
        ros::Duration       pub_interval_;
        ros::Time           pub_last_time_;

        void setCallback()
        {
            mapper_->setCallback(
                        map_t::callback_t::template from<MapperWorker<map_t, msg_t>,
                        &MapperWorker<map_t, msg_t>::publish>(this));
        }

        void setup(
                ros::NodeHandle   & nh,
                const std::string & topic,
                const double      & rate,
                const ros::Time   & now)
        {
            pub_map_       = nh.advertise<msg_t>(topic, 1);
            pub_interval_  = ros::Duration(rate > 0.0 ? 1.0 / rate : 0.0);
            pub_last_time_ = now;
        }

        void requestMap(
                const ros::Time & now)
        {
            if(pub_last_time_.isZero())
                pub_last_time_ = now;

            if (pub_interval_.isZero() || (pub_last_time_ + pub_interval_ < now))
                mapper_->requestMap();
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_2d_t>::value, void>::type
        publish(
                const typename map_t::static_map_stamped_t & map)
        {
            if(map.data()) {
                typename msg_t::Ptr msg;
                cslibs_gridmaps::static_maps::conversion::from(map.data(), msg);
                msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
                msg->header.frame_id = map_frame_;
                pub_map_.publish(msg);
                pub_last_time_ = ros::Time::now();
            }
        }

        template <typename t = msg_t>
        typename std::enable_if<std::is_same<t, msg_3d_t>::value, void>::type
        publish(
                const typename map_t::static_map_stamped_t & map)
        {std::cout << "here " << map.data() << std::endl;
            if (map.data()) {
                typename msg_t::Ptr msg(new msg_t());
                pcl::toROSMsg(*(map.data()), *msg);
                msg->header.stamp.fromNSec(static_cast<uint64_t>(map.stamp().nanoseconds()));
                msg->header.frame_id = map_frame_;
                pub_map_.publish(msg);
                pub_last_time_ = ros::Time::now();
            }
        }
    };

public:
    GridMapperNode();
    bool setup();
    void run();

private:
    ros::NodeHandle                 nh_;
    std::vector<ros::Subscriber>    sub_lasers_;

    // possible maps
    MapperWorker<occ_map_2d_t, msg_2d_t> occ_2d_mapper_;
    MapperWorker<ndt_map_2d_t, msg_2d_t> ndt_2d_mapper_;
    MapperWorker<ndt_map_3d_t, msg_3d_t> ndt_3d_mapper_;

    muse_mcl_2d::TFProvider::Ptr    tf_;
    double                          node_rate_;

    bool                            undistortion_;              /// check if undistortion shall be applied
    std::string                     undistortion_fixed_frame_;  /// the fixed frame necessary for the undistortion
    ros::Duration                   tf_timeout_;                /// time out for the tf listener

    interval_t                      linear_interval_;           /// linear field of view
    interval_t                      angular_interval_;          /// angular field of view

    // 2d and 3d laser callbacks
    void laserscan2d(
            const sensor_msgs::LaserScanConstPtr & msg);

    void laserscan3d(
            const sensor_msgs::PointCloud2ConstPtr & msg);

    template <typename map_t>
    void insert(
            MapperWorker<map_t, msg_2d_t>             & mapper,
            const muse_mcl_2d_laser::LaserScan2D::Ptr & laserscan)
    {
        cslibs_math_2d::Transform2d o_T_l;
        if(tf_->lookupTransform(mapper.map_frame_, laserscan->getFrame(),
                                ros::Time(laserscan->getTimeFrame().end.seconds()),
                                o_T_l,
                                tf_timeout_)) {

            cslibs_math_2d::Pointcloud2d::Ptr points(new cslibs_math_2d::Pointcloud2d);
            muse_mcl_2d_mapping::Measurement2d m(points, o_T_l, cslibs_time::Time(laserscan->getTimeFrame().end));
            for(auto it = laserscan->begin() ; it != laserscan->end() ; ++ it)
                if(it->valid())
                    points->insert(it->point);

            mapper.mapper_->insert(m);
        }
    }

    template <typename map_t, typename point_t>
    void insert(
            MapperWorker<map_t, msg_3d_t>                     & mapper,
            const typename pcl::PointCloud<point_t>::ConstPtr & laserscan)
    {
        tf::Transform o_T_l;
        uint64_t nanoseconds = laserscan->header.stamp * 1e3;
        if (tf_->lookupTransform(mapper.map_frame_,
                                 laserscan->header.frame_id,
                                 ros::Time().fromNSec(nanoseconds),
                                 o_T_l,
                                 tf_timeout_)) {

            cslibs_math_3d::Transform3d origin = cslibs_math_3d::conversion::from(o_T_l);
            cslibs_math_3d::Pointcloud3d::Ptr points(new cslibs_math_3d::Pointcloud3d);
            muse_mcl_3d_mapping::Measurement3d m(points,
                                                 origin,
                                                 cslibs_time::Time(nanoseconds));

            cslibs_time::Time now = cslibs_time::Time::now();
            for(auto it = laserscan->begin() ; it != laserscan->end() ; ++ it) {
                typename map_t::dynamic_map_t::point_t::arr_t arr =  {static_cast<double>(it->x),
                                                                      static_cast<double>(it->y),
                                                                      static_cast<double>(it->z)};
                if (std::isnormal(arr[0]) && std::isnormal(arr[1]) && std::isnormal(arr[2]))
                    points->insert(typename map_t::dynamic_map_t::point_t(arr));
            }
            mapper.mapper_->insert(m);
            std::cout << "Insertion on storage took " << (cslibs_time::Time::now() - now) << std::endl;
        }
    }
};
}
