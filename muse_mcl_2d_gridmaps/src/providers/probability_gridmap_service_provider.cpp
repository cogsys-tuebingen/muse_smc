#include "probability_gridmap_service_provider.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_probability_gridmap.hpp>
#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::ProbabilityGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

ProbabilityGridmapServiceProvider::ProbabilityGridmapServiceProvider() :
    loading_(false)
{
}

void ProbabilityGridmapServiceProvider::setup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_ = nh_private.param<std::string>(param_name("service"), "/static_map");
    source_ = nh_private.serviceClient<nav_msgs::GetMap>(service_name_);
    blocking_ = nh_private.param<bool>(param_name("blocking"), false);
}


ProbabilityGridmapServiceProvider::state_space_t::ConstPtr ProbabilityGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || muse_smc::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                loading_ = true;

                auto load = [this, req]() {
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_ = static_maps::conversion::from(req.response.map);;
                    loading_ = false;
                };
                auto load_blocking = [this, req]() {
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_ = static_maps::conversion::from(req.response.map);
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

