#ifndef MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H

#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <muse_mcl/providers/provider_map.hpp>
#include <muse_mcl_2d_gridmaps/maps//probability_gridmap.h>

namespace muse_mcl {
class ProviderGridmapProbabilityService : public ProviderMap
{
public:
    ProviderGridmapProbabilityService();

    Map::ConstPtr getMap() const override;

protected:
    mutable ros::ServiceClient source_;
    std::string                service_name_;
    bool                       blocking_;

    mutable std::mutex                    map_mutex_;
    mutable std::condition_variable       map_loaded_;
    mutable maps::ProbabilityGridMap::Ptr map_;
    mutable std::atomic_bool              loading_;
    mutable std::thread                   worker_;

    void doSetup(ros::NodeHandle &nh_private) override;

};
}


#endif // MAP_PROVIDER_PROBABILITY_GRIDMAP_SERVICE_H
