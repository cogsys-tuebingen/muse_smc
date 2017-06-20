#include "map_provider_distance_gridmap.h"

#include <opencv2/opencv.hpp>

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(muse_mcl::MapProviderDistanceGridMap, muse_mcl::MapProvider)

using namespace muse_mcl;

MapProviderDistanceGridMap::MapProviderDistanceGridMap() :
    loading_(false)
{
}

Map::ConstPtr MapProviderDistanceGridMap::getMap() const
{
    std::unique_lock<std::mutex> l(map_mutex_);
    if(!map_ && blocking_) {
        map_loaded_.wait(l);
    }

    cv::Mat display(map_->getHeight(), map_->getWidth(), CV_32FC1, cv::Scalar());
    for(int i = 0 ; i < display.rows ; ++i) {
        for(int j = 0 ; j < display.cols ; ++j) {
            display.at<float>(i,j) = map_->at(j,i);
        }
    }

    cv::imshow("map", display);
    cv::waitKey(0);

    return map_;
}

void MapProviderDistanceGridMap::doSetup(ros::NodeHandle &nh_private)
{

    topic_ = nh_private.param<std::string>(privateParameter("topic"), "/map");
    binarization_threshold_ = nh_private.param<double>(privateParameter("threshold"), 0.5);
    kernel_size_ = std::max(nh_private.param<int>(privateParameter("kernel_size"), 5), 5);
    kernel_size_ += 1 - (kernel_size_ % 2);
    blocking_ = nh_private.param<bool>(privateParameter("blocking"), false);

    source_= nh_private.subscribe(topic_, 1, &MapProviderDistanceGridMap::callback, this);

    Logger &l = Logger::getLogger();
    l.info("topic_='" + topic_ + "'", "MapProvider:" + name_);
    l.info("binarization_threshold_='" + std::to_string(binarization_threshold_) + "'", "MapProvider:" + name_);
    l.info("kernel_size_='" + std::to_string(kernel_size_) + "'", "MapProvider:" + name_);
    l.info("blocking_='" + std::to_string(blocking_) + "'", "MapProvider:" + name_);
}

void MapProviderDistanceGridMap::callback(const nav_msgs::OccupancyGridConstPtr &msg)
{
    /// conversion can take time
    /// we allow concurrent loading, this way, the front end thread is not blocking.
    if(!loading_) {
        if(!map_ || msg->info.map_load_time > map_->getStamp()) {
            loading_ = true;

            auto load = [this, msg]() {
                maps::DistanceGridMap::Ptr map(new maps::DistanceGridMap(*msg, binarization_threshold_, kernel_size_));
                std::unique_lock<std::mutex>l(map_mutex_);
                map_ = map;
                loading_ = false;
            };
            auto load_blocking = [this, msg]() {
                std::unique_lock<std::mutex>l(map_mutex_);
                maps::DistanceGridMap::Ptr map(new maps::DistanceGridMap(*msg, binarization_threshold_, kernel_size_));
                map_ = map;
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
