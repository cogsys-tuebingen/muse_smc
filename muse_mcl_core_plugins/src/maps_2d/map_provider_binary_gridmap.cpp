#include "map_provider_binary_gridmap.h"

using namespace muse_mcl;

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::MapProviderBinaryGridMap, muse_mcl::MapProvider)

MapProviderBinaryGridMap::MapProviderBinaryGridMap() :
    loading_(false)
{
}

Map::ConstPtr MapProviderBinaryGridMap::getMap() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }
    return map_;
}

void MapProviderBinaryGridMap::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/map");
    binarization_threshold_ = nh_private.param<double>(privateParameter("threshold"), 0.5);
    source_= nh_private.subscribe(topic_, 1, &MapProviderBinaryGridMap::callback, this);
    blocking_ = nh_private.param<bool>(privateParameter("blocking"), false);
}

void MapProviderBinaryGridMap::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || msg->info.map_load_time > map_->getStamp()) {
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

