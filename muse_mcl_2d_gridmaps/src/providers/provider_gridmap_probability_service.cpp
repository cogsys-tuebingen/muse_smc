#include "provider_gridmap_probability_service.h"

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProviderGridmapProbabilityService, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

ProviderGridmapProbabilityService::ProviderGridmapProbabilityService() :
    loading_(false)
{
}

void ProviderGridmapProbabilityService::setup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_ = nh_private.param<std::string>(param_name("service"), "/static_map");
    source_ = nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
    blocking_ = nh_private.param<bool>(param_name("blocking"), false);
}


ProviderGridmapProbabilityService::state_space_t::ConstPtr ProviderGridmapProbabilityService::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || muse_smc::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
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

