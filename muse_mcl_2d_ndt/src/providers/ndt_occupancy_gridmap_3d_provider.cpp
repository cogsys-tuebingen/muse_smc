#include <muse_mcl_2d_ndt/providers/ndt_occupancy_gridmap_3d_provider.h>

#include <cslibs_ndt_3d/serialization/dynamic_maps/occupancy_gridmap.hpp>
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
    std::unique_lock<std::mutex> l(map_mutex_);
    if (!map_ && blocking_)
        map_loaded_.wait(l);

    return map_;
}

void NDTOccupancyGridmap3dProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};

    path_     = nh.param<std::string>(param_name("path"), "");
    frame_id_ = nh.param<std::string>(param_name("frame_id"), "/world");
    blocking_ = nh.param<bool>(param_name("blocking"), false);

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
}
