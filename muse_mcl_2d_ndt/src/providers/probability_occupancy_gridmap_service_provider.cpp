#include <muse_mcl_2d_ndt/providers/probability_occupancy_gridmap_service_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/occupancy_gridmap.hpp>
#include <cslibs_ndt_2d/conversion/probability_gridmap.hpp>
#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::ProbabilityOccupancyGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
ProbabilityOccupancyGridmapServiceProvider::ProbabilityOccupancyGridmapServiceProvider() :
    loading_(false)
{
}

ProbabilityOccupancyGridmapServiceProvider::state_space_t::ConstPtr ProbabilityOccupancyGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if (source_.call(req))
        loadMap();

    {
        std::unique_lock<std::mutex> l(map_mutex_);
        if (!map_ && blocking_)
            map_loaded_.wait(l);
    }

    publishMap();

    return map_;
}

void ProbabilityOccupancyGridmapServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    service_name_        = nh.param<std::string>(param_name("service"), "/static_map");
    path_                = nh.param<std::string>(param_name("path"), "");
    frame_id_            = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_            = nh.param<bool>(param_name("blocking"), false);
    sampling_resolution_ = nh.param<double>(param_name("sampling_resolution"), 0.05);

    const double prob_prior     = nh.param(param_name("prob_prior"), 0.5);
    const double prob_free      = nh.param(param_name("prob_free"), 0.45);
    const double prob_occupied  = nh.param(param_name("prob_occupied"), 0.65);
    inverse_model_.reset(new cslibs_gridmaps::utility::InverseModel(prob_prior, prob_free, prob_occupied));

    const std::string topic = nh.param<std::string>(param_name("topic"), "/muse_mcl_2d_ndt/occ_map");
    pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic, 1);

    source_ = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}

void ProbabilityOccupancyGridmapServiceProvider::loadMap() const
{
    if (!loading_ && !map_) {
        loading_ = true;

        auto load = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr map;
            if (cslibs_ndt_2d::dynamic_maps::load(map, path_)) {
                std::unique_lock<std::mutex> l(map_mutex_);

                cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr lf_map =
                        cslibs_ndt_2d::conversion::from(map, sampling_resolution_, inverse_model_);
                if (lf_map) {
                    map_.reset(new muse_mcl_2d_gridmaps::ProbabilityGridmap(lf_map, frame_id_));
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
            cslibs_ndt_2d::dynamic_maps::OccupancyGridmap::Ptr map;
            if (cslibs_ndt_2d::dynamic_maps::load(map, path_)) {
                std::unique_lock<std::mutex> l(map_mutex_);

                cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr lf_map =
                        cslibs_ndt_2d::conversion::from(map, sampling_resolution_, inverse_model_);
                if (lf_map) {
                    map_.reset(new muse_mcl_2d_gridmaps::ProbabilityGridmap(lf_map, frame_id_));
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

void ProbabilityOccupancyGridmapServiceProvider::publishMap() const
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
