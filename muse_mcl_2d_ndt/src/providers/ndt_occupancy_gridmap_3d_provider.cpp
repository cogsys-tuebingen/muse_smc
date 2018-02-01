#include <muse_mcl_2d_ndt/providers/ndt_occupancy_gridmap_3d_provider.h>

#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_ndt_3d/conversion/pointcloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::NDTOccupancyGridmap3dProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
NDTOccupancyGridmap3dProvider::NDTOccupancyGridmap3dProvider() :
    loading_(false)
{
}

NDTOccupancyGridmap3dProvider::state_space_t::ConstPtr NDTOccupancyGridmap3dProvider::getStateSpace() const
{
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_ && blocking_)
            map_loaded_.wait(l);
    }

    publishMap();
    return map_;
}

void NDTOccupancyGridmap3dProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_     = nh.param<std::string>(param_name("path"), "");
    frame_id_ = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_ = nh.param<bool>(param_name("blocking"), false);

    const std::string topic = nh.param<std::string>(param_name("topic"), "/muse_mcl_2d_ndt/occ_ndt_3d_map");
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);

    threshold_                  = nh.param(param_name("threshold"), 0.196);
    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));

    loadMap();
}

void NDTOccupancyGridmap3dProvider::loadMap()
{
    if (!loading_ && !map_) {
        loading_ = true;

        auto load = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr map;
            if (cslibs_ndt_3d::dynamic_maps::loadBinary(path_, map)) {
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new OccupancyGridmap3d(map, frame_id_));
                loading_ = false;
                ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
            } else
                ROS_INFO_STREAM("Could not load file '" << path_ << "'!");
        };
        auto load_blocking = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_3d::dynamic_maps::OccupancyGridmap::Ptr map;
            if (cslibs_ndt_3d::dynamic_maps::loadBinary(path_, map)) {
                std::unique_lock<std::mutex> l(map_mutex_);
                map_.reset(new OccupancyGridmap3d(map, frame_id_));
                loading_ = false;
                ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
                map_loaded_.notify_one();
            } else
                ROS_INFO_STREAM("Could not load file '" << path_ << "'!");
        };

        if (blocking_)
            worker_ = std::thread(load_blocking);
        else
            worker_ = std::thread(load);
    }
}

void NDTOccupancyGridmap3dProvider::publishMap() const
{
    if (!map_)
        return;

    if (!msg_) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr prob;
        std::unique_lock<std::mutex> l(map_mutex_);
        cslibs_ndt_3d::conversion::from(map_->data(), prob, inverse_model_, threshold_);
        if (prob) {
            msg_.reset(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*prob, *msg_);
        }
    }

    if (msg_) {
        msg_->header.frame_id = frame_id_;
        msg_->header.stamp    = ros::Time::now();
        pub_.publish(msg_);
    } else
        ROS_INFO_STREAM("Could not publish loaded map!");
}
}
