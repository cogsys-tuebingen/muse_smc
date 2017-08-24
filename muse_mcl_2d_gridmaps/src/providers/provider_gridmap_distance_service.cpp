#include "provider_gridmap_distance_service.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProviderGridmapDistanceService, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

ProviderGridmapDistanceService::ProviderGridmapDistanceService() :
    loading_(false)
{
}

void ProviderGridmapDistanceService::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_ = nh.param<std::string>(param_name("service"), "/static_map");
    binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
    kernel_size_ = std::max(nh.param<int>(param_name("kernel_size"), 5), 5);
    kernel_size_ += 1 - (kernel_size_ % 2);
    blocking_ = nh.param<bool>(param_name("blocking"), false);
    source_   = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}


ProviderGridmapDistanceService::state_space_t::ConstPtr ProviderGridmapDistanceService::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || muse_smc::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
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


