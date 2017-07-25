#ifndef MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H
#define MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H

#include <nav_msgs/OccupancyGrid.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <muse_mcl/plugins/types/provider_map.hpp>
#include <muse_mcl_core_plugins/maps_2d/binary_gridmap.h>

namespace muse_mcl {
class ProviderGridmapBinaryService : public ProviderMap
{
public:
    ProviderGridmapBinaryService();

    Map::ConstPtr getMap() const override;

protected:
    mutable ros::ServiceClient          source_;
    std::string                         service_name_;
    double                              binarization_threshold_;
    bool                                blocking_;

    mutable std::mutex                  map_mutex_;
    mutable std::condition_variable     map_loaded_;
    mutable maps::BinaryGridMap::Ptr    map_;
    mutable std::atomic_bool            loading_;
    mutable std::thread                 worker_;

    void doSetup(ros::NodeHandle &nh_private) override;

};
}


#endif // MAP_PROVIDER_BINARY_GRIDMAP_SERVICE_H
