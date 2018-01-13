#include <muse_mcl_2d_ndt/providers/gridmap_provider.h>

#include <cslibs_ndt_2d/serialization/dynamic_maps/gridmap.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_ndt::GridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_ndt {
GridmapProvider::GridmapProvider() :
    loading_(false)
{
}

GridmapProvider::state_space_t::ConstPtr GridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_ && blocking_)
        map_loaded_.wait(l);

    return map_;
}

void GridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_     = nh.param<std::string>(param_name("path"), "");
    frame_id_ = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_ = nh.param<bool>(param_name("blocking"), false);

    loadMap();
}

void GridmapProvider::loadMap()
{
    if (!loading_ && !map_) {
        loading_ = true;

        auto load = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map =
                YAML::LoadFile(path_).as<cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr>();
            std::unique_lock<std::mutex> l(map_mutex_);
            map_.reset(new Gridmap(map, frame_id_));
            loading_ = false;
        };
        auto load_blocking = [this]() {
            ROS_INFO_STREAM("Loading file '" << path_ << "'...");
            cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr map =
                YAML::LoadFile(path_).as<cslibs_ndt_2d::dynamic_maps::Gridmap::Ptr>();
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
