#include "likelihood_field_gridmap_provider.h"

#include <cslibs_gridmaps/static_maps/conversion/convert_likelihood_field_gridmap.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl_2d_gridmaps::LikelihoodFieldGridmapProvider, muse_mcl_2d::MapProvider2D)

namespace muse_mcl_2d_gridmaps {
LikelihoodFieldGridmapProvider::LikelihoodFieldGridmapProvider() :
    loading_(false)
{
}

LikelihoodFieldGridmapProvider::state_space_t::ConstPtr LikelihoodFieldGridmapProvider::getStateSpace() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }

    return map_;
}

void LikelihoodFieldGridmapProvider::setup(ros::NodeHandle &nh_private)
{
    auto param_name = [this](const std::string &name){return name_ + "/" + name;};
    topic_                  = nh_private.param<std::string>(param_name("topic"), "/map");
    binarization_threshold_ = nh_private.param<double>(param_name("threshold"), 0.5);
    maximum_distance_       = nh_private.param<double>(param_name("maximum_distance"), 2.0);
    z_hit_                  = nh_private.param<double>(param_name("z_hit"), 0.8);
    sigma_hit_              = nh_private.param<double>(param_name("sigma_hit"), 0.2);
    blocking_               = nh_private.param<bool>(param_name("blocking"), false);

    source_= nh_private.subscribe(topic_, 1, &LikelihoodFieldGridmapProvider::callback, this);
}

void LikelihoodFieldGridmapProvider::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || cslibs_time::Time(msg->info.map_load_time.toNSec()) > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map, maximum_distance_, sigma_hit_, binarization_threshold_);
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new LikelihoodFieldGridmap(map, msg->header.frame_id));
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                cslibs_gridmaps::static_maps::LikelihoodFieldGridmap::Ptr map;
                cslibs_gridmaps::static_maps::conversion::from(*msg, map, maximum_distance_, sigma_hit_, binarization_threshold_);
                std::unique_lock<std::mutex>l(map_mutex_);
                map_.reset(new LikelihoodFieldGridmap(map, msg->header.frame_id));
                loading_ = false;
                map_loaded_.notify_one();
            };
            if(blocking_) {
                worker_ = std::thread(load_blocking);
            } else {
                worker_ = std::thread(load);
            }
        }
    }
}
}
