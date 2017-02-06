#ifndef MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H

#include <nav_msgs/GetMap.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <muse_amcl/data_sources/map_provider.hpp>
#include <muse_amcl_core_plugins/maps_2d/distance_gridmap.h>


namespace muse_amcl {
class MapProviderDistanceGridMapService : public MapProvider
{
public:
    MapProviderDistanceGridMapService() = default;

    Map::ConstPtr getMap() const override;

protected:
    mutable ros::ServiceClient source_;
    std::string                service_name_;
    double                     binarization_threshold_;
    std::size_t                kernel_size_;

    mutable std::mutex                 map_mutex_;
    mutable maps::DistanceGridMap::Ptr map_;
    mutable std::atomic_bool           loading_;
    mutable std::thread                worker_;

    void doSetup(ros::NodeHandle &nh_private) override;
};
}

#endif // MAP_PROVIDER_DISTANCE_GRIDMAP_SERVICE_H
