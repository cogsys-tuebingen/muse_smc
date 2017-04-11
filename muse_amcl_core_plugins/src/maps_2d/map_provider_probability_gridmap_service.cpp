#include "map_provider_probability_gridmap_service.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_amcl::MapProviderProbabilityGridMapService, muse_amcl::MapProvider)

using namespace muse_amcl;

MapProviderProbabilityGridMapService::MapProviderProbabilityGridMapService() :
    loading_(false)
{
}

Map::ConstPtr MapProviderProbabilityGridMapService::getMap() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || req.response.map.info.map_load_time > map_->getStamp()) {
                loading_ = true;

                auto load = [this, req]() {
                    maps::ProbabilityGridMap::Ptr map(new maps::ProbabilityGridMap(req.response.map));
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_ = map;
                    loading_ = false;
                };
                auto load_blocking = [this, req]() {
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_.reset(new maps::ProbabilityGridMap(req.response.map));
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
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }
    return map_;

}

void MapProviderProbabilityGridMapService::doSetup(ros::NodeHandle &nh_private)
{
    service_name_ = nh_private.param<std::string>(privateParameter("service"), "/static_map");
    source_ = nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
    blocking_ = nh_private.param<bool>(privateParameter("blocking"), false);

    Logger &l = Logger::getLogger();
    l.info("service_name_='" + service_name_ + "'", "MapProvider:" + name_);
    l.info("blocking_='" + std::to_string(blocking_) + "'", "MapProvider:" + name_);
}
