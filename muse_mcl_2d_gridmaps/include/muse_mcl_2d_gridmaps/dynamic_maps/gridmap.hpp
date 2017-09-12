#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>


#include <muse_mcl_2d/map/map_2d.hpp>
#include <muse_mcl_2d_gridmaps/dynamic_maps/algorithms/bresenham.hpp>
#include <muse_mcl_2d_gridmaps/dynamic_maps/chunk.hpp>

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
    using index_t = std::array<int, 2>;
    using chunk_t = Chunk<T>;
    using cis_kd_tree_t = cis::Storage<chunk_t, index_t, cis::backend::kdtree::KDTree>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::string &frame) :
        Map2D(frame),
        resolution_(resolution),
        resolution_inv_(1.0 / resolution_),
        w_T_m_(origin_x, origin_y, origin_phi),
        minimum_index_{std::numeric_limits<int>::max(),
                       std::numeric_limits<int>::max()},
        maximum_index_{std::numeric_limits<int>::min(),
                       std::numeric_limits<int>::min()}
    {
    }

    virtual inline muse_mcl_2d::Point2D getMin() const override
    {
        muse_mcl_2d::Point2D p;
        fromIndex({0,0},p);
        return p;
    }

    virtual inline muse_mcl_2d::Point2D getMax() const override
    {
        muse_mcl_2d::Point2D p;
        fromIndex({(int)width_-1,(int)height_-1},p);
        return p;
    }



protected:
    const double                resolution_;
    const double                resolution_inv_;
    muse_mcl_2d::Transform2D    w_T_m_;
    muse_mcl_2d::Transform2D    m_T_w_;

    index_t                     min_index_;
    index_t                     max_index_;


};
}
}



#endif // GRIDMAP_HPP
