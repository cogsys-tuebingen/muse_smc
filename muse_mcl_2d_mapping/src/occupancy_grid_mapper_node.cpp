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
    const std::size_t   subscriber_queue_size       = nh_.param<int>("subscriber_queue_size", 1);
    const double        occ_grid_resolution         = nh_.param<double>("occ_grid_resolution", 0.05);
    const double        occ_grid_chunk_resolution   = nh_.param<double>("occ_chunk_resolution", 5.0);
    const std::string   occ_map_topic               = nh_.param<std::string>("occ_map_topic", "/map");
    const double        occ_map_pub_rate            = nh_.param<double>("occ_map_pub_rate", 10.0);
    const double        occ_map_prob_free           = nh_.param<double>("occ_map_prob_free", 0.45);
    const double        occ_map_prob_occ            = nh_.param<double>("occ_map_prob_occ", 0.55);
    const double        occ_map_prob_prior          = nh_.param<double>("occ_map_prob_prior", 0.5);

    std::vector<std::string> lasers;
    if(!nh_.getParam("lasers", lasers)) {
        ROS_ERROR_STREAM("Did not find any laser inputs!");
        return false;
    }

    map_frame_                = nh_.param<std::string>("map_frame", "/odom");

    rate_                     = nh_.param<double>("rate", 0.0);
    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");

    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "");
    tf_timeout_               = ros::Duration(nh_.param("tf_timeout", 0.1));

    linear_interval_[0]  = nh_.param<double>("range_min", 0.05);
    linear_interval_[1]  = nh_.param<double>("range_max", 30.0);
    angular_interval_[0] = nh_.param<double>("angle_min",-M_PI);
    angular_interval_[1] = nh_.param<double>("angle_max", M_PI);


    muse_mcl_2d_gridmaps::mapping::InverseModel inverse_model(occ_map_prob_prior, occ_map_prob_free, occ_map_prob_occ);
    mapper_.reset(new OccupancyGridMapper(inverse_model,
                                          occ_grid_resolution,
                                          occ_grid_chunk_resolution,
                                          map_frame_));


    for(const auto &l : lasers) {
        sub_lasers_.emplace_back(nh_.subscribe(l,
                                               subscriber_queue_size,
                                               &OccupancyGridMapperNode::laserscan,
                                               this));
    }
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>(occ_map_topic, 1);

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
            !convertUndistorted(msg, linear_interval_, angular_interval_, tf_, undistortion_fixed_frame_, tf_timeout_, laserscan)) {
        if(!convert(msg, linear_interval_, angular_interval_, laserscan)) {
            return;
        }
    } else if(!convert(msg, linear_interval_, angular_interval_, laserscan)) {
        return;
    }

    muse_mcl_2d::math::Transform2D m_T_l;
    if(tf_->lookupTransform(laserscan->getFrame(), map_frame_,
                            ros::Time(laserscan->getTimeFrame().end.seconds()),
                            m_T_l,
                            tf_timeout_)) {

        Pointcloud2D::Ptr points(new Pointcloud2D(laserscan->getFrame(),
                                                  laserscan->getTimeFrame(),
                                                  m_T_l));
        for(auto it = laserscan->begin() ; it != laserscan->end() ; ++it) {
            if(it->valid())
                points->insert(m_T_l * it->point);
        }
        mapper_->insert(points);
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d_mapping_ocm_node");


    return 0;
}
