#include "occupancy_grid_mapper_node.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
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

    node_rate_                = nh_.param<double>("rate", 0.0);
    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");

    undistortion_             = nh_.param<bool>("undistortion", true);
    undistortion_fixed_frame_ = nh_.param<std::string>("undistortion_fixed_frame", "/odom");
    tf_timeout_               = ros::Duration(nh_.param("tf_timeout", 0.1));

    linear_interval_[0]  = nh_.param<double>("range_min", 0.05);
    linear_interval_[1]  = nh_.param<double>("range_max", 30.0);
    angular_interval_[0] = nh_.param<double>("angle_min",-M_PI);
    angular_interval_[1] = nh_.param<double>("angle_max", M_PI);


    muse_mcl_2d_gridmaps::utility::InverseModel inverse_model(occ_map_prob_prior, occ_map_prob_free, occ_map_prob_occ);
    occ_mapper_.reset(new OccupancyGridMapper(inverse_model,
                                              occ_grid_resolution,
                                              occ_grid_chunk_resolution,
                                              map_frame_));


    sub_laser_ = nh_.subscribe("/sick/front",
                               subscriber_queue_size,
                               &OccupancyGridMapperNode::laserscan,
                               this);
    for(const auto &l : lasers) {
        ROS_INFO_STREAM("Subscribing to laser '" << l << "'");
        sub_lasers_.emplace_back(std::move(nh_.subscribe(l,
                                                         subscriber_queue_size,
                                                         &OccupancyGridMapperNode::laserscan,
                                                         this)));
    }
    pub_occ_map_        = nh_.advertise<nav_msgs::OccupancyGrid>(occ_map_topic, 1);
    pub_occ_interval_   = ros::Duration(occ_map_pub_rate > 0.0 ? 1.0 / occ_map_pub_rate : 0.0);
    pub_occ_last_time_  = ros::Time::now();

    tf_.reset(new muse_mcl_2d::TFProvider);

    ROS_INFO_STREAM("Setup succesful!");
    return true;
}

void OccupancyGridMapperNode::run()
{
    if(node_rate_ == 0.0) {
        while(ros::ok()) {
            const ros::Time now = ros::Time::now();
            if(pub_occ_interval_.isZero() || (pub_occ_last_time_ + pub_occ_interval_ < now)) {
                publishOcc();
                pub_occ_last_time_ = now;
            }
            ros::spinOnce();
        }
    } else {
        ros::Rate r(node_rate_);
        while(ros::ok()) {
            const ros::Time now = ros::Time::now();
            if(pub_occ_interval_.isZero() || pub_occ_last_time_ + pub_occ_interval_ < now) {
                publishOcc();
                pub_occ_last_time_ = now;
            }
            r.sleep();
            ros::spinOnce();
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

    cslibs_math_2d::Transform2d m_T_l;
    if(tf_->lookupTransform(laserscan->getFrame(), map_frame_,
                            ros::Time(laserscan->getTimeFrame().end.seconds()),
                            m_T_l,
                            tf_timeout_)) {

        cslibs_math_2d::Pointcloud2d::Ptr points(new cslibs_math_2d::Pointcloud2d);
        OccupancyGridMapper::Measurement  m(points, m_T_l);
        for(auto it = laserscan->begin() ; it != laserscan->end() ; ++it) {
            if(it->valid())
                points->insert(m_T_l * it->point);
        }
        occ_mapper_->insert(m);
    }
}

void OccupancyGridMapperNode::publishOcc()
{
    auto map = occ_mapper_->get();
    if(map) {


        nav_msgs::OccupancyGrid::Ptr msg;
        muse_mcl_2d_gridmaps::static_maps::conversion::from(map, msg);
        pub_occ_map_.publish(msg);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "muse_mcl_2d_mapping_ocm_node");
    OccupancyGridMapperNode instance;
    instance.setup();
    instance.run();

    return 0;
}
