#include "occupancy_grid_mapper_node.h"

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
    const double        resolution = nh_.param<double>("resolution", 0.05);
    const std::string   map_topic = nh_.param<std::string>("map_topic", "/map");
    const std::string   map_frame = nh_.param<std::string>("map_frame", "/odom");
    const double        map_pub_rate = nh_.param<double>("map_pub_rate", 10.0);
    std::vector<std::string> lasers;
    if(!nh_.getParam("lasers", lasers)) {
        ROS_ERROR_STREAM("Did not find any laser inputs!");
        return false;
    }

    for(const auto &l : lasers) {
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               subscriber_queue_size,
                                               &OccupancyGridMapperNode::laserscan,
                                               this));
    }
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);


    rate_ = nh_.param<double>("rate", 0.0);

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

}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d_mapping_ocm_node");


    return 0;
}
