#include <muse_mcl_2d_ndt/providers/binary_gridmap_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_ndt_2d/conversion/binary_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/OccupancyGrid.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::BinaryGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
BinaryGridmapProvider::BinaryGridmapProvider() :
    loading_(false)
{
}

BinaryGridmapProvider::state_space_t::ConstPtr BinaryGridmapProvider::getStateSpace() const
{
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_ && blocking_)
            map_loaded_.wait(l);
    }

    publishMap();

    return map_;
}

void BinaryGridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_                = nh.param<std::string>(param_name("path"), "");
    frame_id_            = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_            = nh.param<bool>(param_name("blocking"), false);

    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.05);
    threshold_           = nh.param<double>(param_name("threshold"), 0.196);

    const std::string topic = nh.param<std::string>(param_name("topic"), "/muse_mcl_2d_ndt/binary_ndt_map");
    pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);

    loadMap();
}

void BinaryGridmapProvider::loadMap()
{
    if (!loading_ && !map_) {
        loading_ = true;

        auto load = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map;
            if (cslibs_ndt_2d::dynamic_maps::loadBinary(path_, map)) {
                std::unique_lock<std::mutex> l(map_mutex_);

                cslibs_gridmaps::static_maps::BinaryGridmap::Ptr lf_map;
                cslibs_ndt_2d::conversion::from(map, lf_map, sampling_resolution_, threshold_);
                if (lf_map) {
                    map_.reset(new muse_mcl_2d_gridmaps::BinaryGridmap(lf_map, frame_id_));
                    loading_ = false;
                    ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
                    map_loaded_.notify_one();
                } else
                    ROS_INFO_STREAM("Could not convert map to Likelihood Field map");
            } else
                ROS_INFO_STREAM("Could not load file '" << path_ << "'!");
        };
        auto load_blocking = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map;
            if (cslibs_ndt_2d::dynamic_maps::loadBinary(path_, map)) {
                std::unique_lock<std::mutex> l(map_mutex_);

                cslibs_gridmaps::static_maps::BinaryGridmap::Ptr lf_map;
                cslibs_ndt_2d::conversion::from(map, lf_map, sampling_resolution_, threshold_);
                if (lf_map) {
                    map_.reset(new muse_mcl_2d_gridmaps::BinaryGridmap(lf_map, frame_id_));
                    loading_ = false;
                    ROS_INFO_STREAM("Successfully loaded file '" << path_ << "'!");
                    map_loaded_.notify_one();
                } else
                    ROS_INFO_STREAM("Could not convert map to Likelihood Field map");
            } else
                ROS_INFO_STREAM("Could not load file '" << path_ << "'!");
        };

        if (blocking_)
            worker_ = std::thread(load_blocking);
        else
            worker_ = std::thread(load);
    }
}

void BinaryGridmapProvider::publishMap() const
{
    if (!map_)
        return;

    nav_msgs::OccupancyGrid::Ptr msg;
    {
        std::unique_lock<std::mutex> l(map_mutex_);
        cslibs_gridmaps::static_maps::conversion::from(map_->data(), msg);
    }

    if (msg) {
        msg->header.frame_id = frame_id_;
        msg->header.stamp    = ros::Time::now();
        pub_.publish(msg);
    } else
        ROS_INFO_STREAM("Could not publish loaded map!");
}
}
