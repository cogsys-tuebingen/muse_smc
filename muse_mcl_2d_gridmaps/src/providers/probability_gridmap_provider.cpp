#include "probability_gridmap_provider.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapProvider, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

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
                std::unique_lock<std::mutex>l(map_mutex_);
                static_maps::conversion::from(*msg, map_);
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                static_maps::conversion::from(*msg, map_);
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
