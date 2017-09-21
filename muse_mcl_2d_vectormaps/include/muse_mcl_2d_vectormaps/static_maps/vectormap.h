#ifndef VECTORMAP_H
#define VECTORMAP_H

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_vectormaps/maps/vector_map.h>

namespace muse_mcl_2d_vectormaps {
namespace static_maps {

class VectorMap : public muse_mcl_2d::Map2D {
public:
    using Ptr = std::shared_ptr<VectorMap>;

    VectorMap(cslibs_vectormaps::VectorMap::Ptr vector_map) :
        Map2D("map"),
        vector_map_(vector_map)
    {
    }

    virtual muse_mcl_2d::math::Point2D getMin() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->minCorner();
        return muse_mcl_2d::math::Point2D(p.x(), p.y());
    }

    virtual muse_mcl_2d::math::Point2D getMax() const override
    {
        cslibs_vectormaps::VectorMap::Point p = vector_map_->maxCorner();
        return muse_mcl_2d::math::Point2D(p.x(), p.y());
    }

    cslibs_vectormaps::VectorMap &getMap() const
    {
        return *vector_map_;
    }

protected:
    cslibs_vectormaps::VectorMap::Ptr vector_map_;
};

}
}

#endif // VECTOR_MAP_H
