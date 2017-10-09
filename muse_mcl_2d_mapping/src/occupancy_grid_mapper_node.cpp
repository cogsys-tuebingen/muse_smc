#include "occupancy_grid_mapper_node.h"

#include <muse_mcl_2d_laser/convert.hpp>

#include <nav_msgs/OccupancyGrid.h>

using namespace muse_mcl_2d_mapping;

OccupancyGridMapperNode::OccupancyGridMapperNode() :
    nh_("~")
{
}

bool OccupancyGridMapperNode::setup()
{
    ROS_INFO_STREAM("Setting up subscribers");
    const std::size_t   subscriber_queue_size = nh_.param<int>("subscriber_queue_size", 1);
    const double        resolution            = nh_.param<double>("resolution", 0.05);
    const std::string   map_topic             = nh_.param<std::string>("map_topic", "/map");
    const std::string   map_frame             = nh_.param<std::string>("map_frame", "/odom");
    const double        map_pub_rate          = nh_.param<double>("map_pub_rate", 10.0);
    std::vector<std::string> lasers;
    if(!nh_.getParam("lasers", lasers)) {
        ROS_ERROR_STREAM("Did not find any laser inputs!");
        return false;
    }
    rate_                     = nh_.param<double>("rate", 0.0);
    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");

    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "");
    undistortion_tf_timeout_  = ros::Duration(nh_.param("undistotion_tf_timeout", 0.1));

    linear_interval_[0]  = nh_.param<double>("range_min", 0.05);
    linear_interval_[1]  = nh_.param<double>("range_max", 30.0);
    angular_interval_[0] = nh_.param<double>("angle_min",-M_PI);
    angular_interval_[1] = nh_.param<double>("angle_max", M_PI);


    for(const auto &l : lasers) {
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               subscriber_queue_size,
                                               &OccupancyGridMapperNode::laserscan,
                                               this));
    }
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);

    tf_.reset(new muse_mcl_2d::TFProvider);

    ROS_INFO_STREAM("Setup succesfull!");
    return true;
}

void OccupancyGridMapperNode::run()
{
    auto update_map = [this] () {

    };

    if(rate_ == 0.0) {
        while(ros::ok()) {

        }
    } else {
        ros::Rate r(rate_);
        while(ros::ok()) {


            r.sleep();
        }
    }

}

void OccupancyGridMapperNode::laserscan(const sensor_msgs::LaserScanConstPtr &msg)
{
    muse_mcl_2d_laser::LaserScan2D::Ptr laserscan;
    if(undistortion_ &&
            !convertUndistorted(msg, linear_interval_, angular_interval_, tf_, undistortion_fixed_frame_, undistortion_tf_timeout_, laserscan)) {
        if(!convert(msg, linear_interval_, angular_interval_, laserscan)) {
            return;
        }
    } else if(!convert(msg, linear_interval_, angular_interval_, laserscan)) {
        return;
    }

    Pointcloud2D::Ptr points(new Pointcloud2D(laserscan->getFrame(),
                                              laserscan->getTimeFrame()));
    for(auto it = laserscan.begin() ; it != laserscan.end() ; ++it) {
        if(it->isValid())
            points->insert(it->point);
    }


}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d_mapping_ocm_node");


    return 0;
}
