#ifndef DATA_PROVIDER_PROBABILITY_GRID_MAP_H
#define DATA_PROVIDER_PROBABILITY_GRID_MAP_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <muse_mcl/plugins/types/provider_map.hpp>
#include <muse_mcl_2d_gridmaps/maps//probability_gridmap.h>

namespace muse_mcl {
class ProviderGridmapProbability : public ProviderMap
{
public:
    ProviderGridmapProbability();

    Map::ConstPtr getMap() const override;

protected:
    ros::Subscriber source_;
    std::string     topic_;
    bool            blocking_;

    mutable std::mutex               map_mutex_;
    mutable std::condition_variable  map_loaded_;
    maps::ProbabilityGridMap::Ptr    map_;
    std::atomic_bool                 loading_;
    std::thread                      worker_;

    void doSetup(ros::NodeHandle &nh_private) override;
    void callback(const nav_msgs::OccupancyGridConstPtr &msg);


};
}

#endif // DATA_PROVIDER_PROBABILITY_GRID_MAP_H
