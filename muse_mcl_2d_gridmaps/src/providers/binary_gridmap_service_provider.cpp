#include "binary_gridmap_service_provider.h"

#include <muse_mcl_2d_gridmaps/static_maps/conversion/convert_binary_gridmap.hpp>

#include <nav_msgs/GetMap.h>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::BinaryGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

using namespace muse_mcl_2d_gridmaps;
using namespace muse_mcl_2d;

BinaryGridmapServiceProvider::BinaryGridmapServiceProvider() :
    loading_(false)
{
}

void BinaryGridmapServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_ = nh.param<std::string>(param_name("service"), "/static_map");
    binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
    source_= nh.serviceClient<nav_msgs::GetMap>(service_name_);
    blocking_ = nh.param<double>(param_name("blocking"), false);
}


BinaryGridmapServiceProvider::state_space_t::ConstPtr BinaryGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || cslibs_time::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                loading_ = true;

                auto load = [this, req]() {
                    std::unique_lock<std::mutex>l(map_mutex_);
                    static_maps::conversion::from(req.response.map, map_, binarization_threshold_);
                    loading_ = false;
                };
                auto load_blocking = [this, req]() {
                   std::unique_lock<std::mutex> l(map_mutex_);
                   static_maps::conversion::from(req.response.map, map_, binarization_threshold_);
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

