#include "map_provider_binary_gridmap_service.h"

#include <nav_msgs/GetMap.h>

using namespace muse_amcl;

Map::ConstPtr MapProviderBinaryGridMapService::getMap() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || req.response.map.info.map_load_time > map_->getStamp()) {
                loading_ = true;
                auto load = [this, req]() {
                    maps::BinaryGridMap::Ptr map(new maps::BinaryGridMap(req.response.map, binarization_threshold_));
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_ = map;
                    loading_ = false;
                };
                worker_ = std::thread(load);
                worker_.detach();
            }
        }
    }
    std::unique_lock<std::mutex> l(map_mutex_);
    return map_;
}

void MapProviderBinaryGridMapService::doSetup(ros::NodeHandle &nh_private)
{
    service_name_ = nh_private.param<std::string>(privateParameter("service"), "/map");
    binarization_threshold_ = nh_private.param<double>(privateParameter("threshold"), 0.5);
    source_= nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
}
