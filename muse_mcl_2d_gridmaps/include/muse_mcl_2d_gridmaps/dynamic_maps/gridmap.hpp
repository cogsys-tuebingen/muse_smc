#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>



#include <muse_mcl_2d/map/map_2d.hpp>
#include <muse_mcl_2d_gridmaps/dynamic_maps/algorithms/bresenham.hpp>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
template<typename T>
class GridMap : public muse_mcl_2d::Map2D
{
public:
    using Ptr = std::shared_ptr<GridMap<T>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::string &frame) :
        Map2D(frame)
    {
    }

protected:
    const double      resolution_;
    const double      resolution_inv_;
    std::size_t       width_;
    std::size_t       height_;

    double            origin_x_;

};
}
}



#endif // GRIDMAP_HPP
