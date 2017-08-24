#include "binary_gridmap_provider.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BinaryGridmapProvider, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

BinaryGridmapProvider::BinaryGridmapProvider() :
    loading_(false)
{
}

BinaryGridmapProvider::state_space_t::ConstPtr BinaryGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }
    return map_;
}

void BinaryGridmapProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    topic_ = nh.param<std::string>(param_name("topic"), "/map");
    binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
    source_= nh.subscribe(topic_, 1, &BinaryGridmapProvider::callback, this);
    blocking_ = nh.param<bool>(param_name("blocking"), false);
}

void BinaryGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || muse_smc::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                maps::BinaryGridMap::Ptr map(new maps::BinaryGridMap(msg, binarization_threshold_));
                std::unique_lock<std::mutex>l(map_mutex_);
                map_ = map;
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new maps::BinaryGridMap(msg, binarization_threshold_));
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

