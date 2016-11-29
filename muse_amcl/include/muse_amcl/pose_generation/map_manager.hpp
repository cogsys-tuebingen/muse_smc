#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP

#include <memory>
#include <map>
#include <vector>
#include <muse_amcl/data_sources/map_provider.hpp>

namespace muse_amcl {
/**
 * @brief The MapManager class is a helper which allows to execute certain
 *        queries to a bundle of maps, such as finding the maximum dimension.
 *        This helper is useful for pose generation tasks.
 */
class MapManager {
public:
    std::shared_ptr<MapManager> Ptr;

    MapManager(const std::vector<MapProvider::Ptr> providers) :
        providers_(providers)
    {
    }







private:
    std::vector<MapProvider::Ptr> providers_;


};
}

#endif /* MAP_MANGER_HPP */
