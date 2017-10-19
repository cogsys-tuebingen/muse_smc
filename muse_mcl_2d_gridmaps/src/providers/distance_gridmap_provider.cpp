#include "distance_gridmap_provider.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_distance_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::DistanceGridmapProvider, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

DistanceGridmapProvider::DistanceGridmapProvider() :
    loading_(false)
{
}

DistanceGridmapProvider::state_space_t::ConstPtr DistanceGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }

    return map_;
}

void DistanceGridmapProvider::setup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    topic_                  = nh_private.param<std::string>(param_name("topic"), "/map");
    binarization_threshold_ = nh_private.param<double>(param_name("threshold"), 0.5);
    maximum_distance_       = nh_private.param<double>(param_name("maximum_distance"), 2.0);
    blocking_               = nh_private.param<bool>(param_name("blocking"), false);

    source_= nh_private.subscribe(topic_, 1, &DistanceGridmapProvider::callback, this);
}

void DistanceGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || muse_smc::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                map_ = muse_mcl_2d_gridmaps::static_maps::conversion::from(*msg, binarization_threshold_, maximum_distance_);
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                map_ = muse_mcl_2d_gridmaps::static_maps::conversion::from(*msg, binarization_threshold_, maximum_distance_);
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
