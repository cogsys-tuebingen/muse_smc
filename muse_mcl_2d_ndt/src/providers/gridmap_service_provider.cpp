#include <muse_mcl_2d_ndt/providers/gridmap_service_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::GridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
GridmapServiceProvider::GridmapServiceProvider() :
    loading_(false)
{
}

GridmapServiceProvider::state_space_t::ConstPtr GridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if (source_.call(req))
        loadMap();

    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_ && blocking_)
        map_loaded_.wait(l);

    return map_;
}

void GridmapServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    service_name_ = nh.param<std::string>(param_name("service"), "/static_map");
    path_         = nh.param<std::string>(param_name("path"), "");
    frame_id_     = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_     = nh.param<bool>(param_name("blocking"), false);
    source_       = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}

void GridmapServiceProvider::loadMap() const
{
    if (!loading_ && !map_) {
        loading_ = true;

        auto load = [this]() {
            std::ifstream map_in_yaml(path_);
            if (!map_in_yaml.is_open())
                std::cout << "[GridmapServiceProvider]: Could not open file '" << path_ << "'" << std::endl;
            else
                std::cout << "[GridmapServiceProvider]: Loading file '" << path_ << "'..." << std::endl;

            YAML::Node n = YAML::LoadFile(path_);
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map =
                n.as<cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr>();
            std::unique_lock<std::mutex> l(map_mutex_);
            map_.reset(new Gridmap(map, frame_id_));
            loading_ = false;
        };
        auto load_blocking = [this]() {
            std::ifstream map_in_yaml(path_);
            if (!map_in_yaml.is_open())
                std::cout << "[GridmapServiceProvider]: Could not open file '" << path_ << "'" << std::endl;
            else
                std::cout << "[GridmapServiceProvider]: Loading file '" << path_ << "'..." << std::endl;

            YAML::Node n = YAML::LoadFile(path_);
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map =
                n.as<cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr>();
            std::unique_lock<std::mutex> l(map_mutex_);
            map_.reset(new Gridmap(map, frame_id_));
            loading_ = false;
            map_loaded_.notify_one();
        };

        if (blocking_)
            worker_ = std::thread(load_blocking);
        else
            worker_ = std::thread(load);
    }
}
}
