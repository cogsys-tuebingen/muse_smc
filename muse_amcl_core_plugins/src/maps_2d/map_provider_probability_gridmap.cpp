#include "map_provider_probability_gridmap.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::MapProviderProbabilityGridMap, muse_amcl::MapProvider)

using namespace muse_amcl;

MapProviderProbabilityGridMap::MapProviderProbabilityGridMap() :
    loading_(false)
{
}

Map::ConstPtr MapProviderProbabilityGridMap::getMap() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }
    return map_;
}

void MapProviderProbabilityGridMap::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/map");
    source_= nh_private.subscribe(topic_, 1, &MapProviderProbabilityGridMap::callback, this);
    blocking_ = nh_private.param<bool>(privateParameter("blocking"), false);
}

void MapProviderProbabilityGridMap::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || msg->info.map_load_time > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                maps::ProbabilityGridMap::Ptr map(new maps::ProbabilityGridMap(msg));
                std::unique_lock<std::mutex>l(map_mutex_);
                map_ = map;
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new maps::ProbabilityGridMap(msg));
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
