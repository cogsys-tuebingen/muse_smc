#include "map_provider_distance_gridmap_service.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::MapProviderDistanceGridMapService, muse_mcl::MapProvider)

using namespace muse_mcl;

MapProviderDistanceGridMapService::MapProviderDistanceGridMapService() :
    loading_(false)
{
}

Map::ConstPtr MapProviderDistanceGridMapService::getMap() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || req.response.map.info.map_load_time > map_->getStamp()) {
                loading_ = true;

                auto load = [this, req]() {
                    maps::DistanceGridMap::Ptr map(new maps::DistanceGridMap(req.response.map, binarization_threshold_, kernel_size_));
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_ = map;
                    loading_ = false;
                };
                auto load_blocking = [this, req]() {
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_.reset(new maps::DistanceGridMap(req.response.map, binarization_threshold_, kernel_size_));
                    loading_ = false;
                    map_loaded_.notify_one();
                };

                if(blocking_) {
                    worker_ = std::thread(load_blocking);
                } else {
                    worker_ = std::thread(load);
                }
                worker_.detach();
            } else if(blocking_) {
                map_loaded_.notify_one();
            }
        }
    }
    std::unique_lock<std::mutex> l(map_mutex_);
    if(blocking_) {
        map_loaded_.wait(l);
    }
    return map_;

}

void MapProviderDistanceGridMapService::doSetup(ros::NodeHandle &nh_private)
{
    service_name_ = nh_private.param<std::string>(privateParameter("service"), "/static_map");
    binarization_threshold_ = nh_private.param<double>(privateParameter("threshold"), 0.5);
    kernel_size_ = std::max(nh_private.param<int>(privateParameter("kernel_size"), 5), 5);
    kernel_size_ += 1 - (kernel_size_ % 2);
    blocking_ = nh_private.param<bool>(privateParameter("blocking"), false);
    source_   = nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
}

