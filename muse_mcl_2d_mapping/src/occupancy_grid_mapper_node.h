#ifndef OCCUPANCY_GRID_MAPPER_NODE_H
#define OCCUPANCY_GRID_MAPPER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace muse_mcl_2d_mapping {
class OccupancyGridMapperNode
{
public:
    OccupancyGridMapperNode();

    bool setup();
    void run();

private:
    ros::NodeHandle nh_;
    std::vector<ros::Subscriber> sub_lasers_;
    ros::Publisher               pub_map_;

    double rate_;

    void laserscan(const sensor_msgs::LaserScanConstPtr &msg);

};
}

#endif // OCCUPANCY_GRID_MAPPER_NODE_H
