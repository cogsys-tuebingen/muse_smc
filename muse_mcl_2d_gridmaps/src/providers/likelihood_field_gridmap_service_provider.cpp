#include "likelihood_field_gridmap_service_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_likelihood_field_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldGridmapServiceProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldGridmapServiceProvider::LikelihoodFieldGridmapServiceProvider() :
    loading_(false)
{
}

void LikelihoodFieldGridmapServiceProvider::setup(ros::NodeHandle &nh)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    service_name_           = nh.param<std::string>(param_name("service"), "/static_map");
    binarization_threshold_ = nh.param<double>(param_name("threshold"), 0.5);
    maximum_distance_       = nh.param<double>(param_name("maximum_distance"), 2.0);
    sigma_hit_              = nh.param<double>(param_name("sigma_hit"), 0.2);
    blocking_               = nh.param<bool>(param_name("blocking"), false);
    source_                 = nh.serviceClient<nav_msgs::GetMap>(service_name_);
}


LikelihoodFieldGridmapServiceProvider::state_space_t::ConstPtr LikelihoodFieldGridmapServiceProvider::getStateSpace() const
{
    nav_msgs::GetMap req;
    if(source_.call(req)) {
        /// conversion can take time
        /// we allow concurrent loading, this way, the front end thread is not blocking.
        if(!loading_) {
            if(!map_ || cslibs_time::Time(req.response.map.info.map_load_time.toNSec()) > map_->getStamp()) {
                loading_ = true;

                auto load = [this, req]() {
                    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(req.response.map, map, maximum_distance_, sigma_hit_, binarization_threshold_);
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_.reset(new LikelihoodFieldGridmap(map, req.response.map.header.frame_id));
                    loading_ = false;
                };
                auto load_blocking = [this, req]() {
                    cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr map;
                    cslibs_gridmaps::static_maps::conversion::from(req.response.map, map, maximum_distance_, sigma_hit_, binarization_threshold_);
                    std::unique_lock<std::mutex>l(map_mutex_);
                    map_.reset(new LikelihoodFieldGridmap(map, req.response.map.header.frame_id));
                    loading_ = false;
                    map_loaded_.notify_one();
                };

                if(blocking_) {
                    worker_ = std::thread(load_blocking);
                } else {
                    worker_ = std::thread(load);
                }
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
}
