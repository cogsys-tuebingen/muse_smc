#include "probability_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
ProbabilityGridmapProvider::ProbabilityGridmapProvider() :
    loading_(false)
{
}

ProbabilityGridmapProvider::state_space_t::ConstPtr ProbabilityGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }
    return map_;
}

void ProbabilityGridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    topic_ = nh.param<std::string>(param_name("topic"), "/map");
    source_= nh.subscribe(topic_, 1, &ProbabilityGridmapProvider::callback, this);
    blocking_ = nh.param<bool>(param_name("blocking"), false);
}

void ProbabilityGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ ||  cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map);
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new ProbabilityGridmap(map, msg->header.frame_id));
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                cslibs_gridmaps::static_maps::ProbabilityGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map);
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new ProbabilityGridmap(map, msg->header.frame_id));
                loading_ = false;
                map_loaded_.notify_one();
            };

            if(blocking_) {
                worker_ = std::thread(load_blocking);
            } else {
                worker_ = std::thread(load);
            }
            worker_.detach();
        }
    }
}
}
