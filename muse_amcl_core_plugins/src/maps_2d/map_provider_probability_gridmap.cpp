#include "map_provider_probability_gridmap.h"

using namespace muse_amcl;

Map::ConstPtr MapProviderProbabilityGridMap::getMap() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void MapProviderProbabilityGridMap::doSetup(ros::NodeHandle &nh_private)
{
    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/map");
    source_= nh_private.subscribe(topic_, 1, &MapProviderProbabilityGridMap::callback, this);
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
            worker_ = std::thread(load);
            worker_.detach();
        }
    }
}
