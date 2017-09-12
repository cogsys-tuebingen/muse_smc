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
    using Ptr       = std::shared_ptr<GridMap<T>>;
    using index_t   = std::array<int, 2>;
    using chunk_t   = Chunk<T>;
    using storage_t = cis::Storage<chunk_t, index_t, cis::backend::kdtree::KDTree>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const double chunk_resolution,
            const T &default_value,
            const std::string &frame) :
        Map2D(frame),
        resolution_(resolution),
        resolution_inv_(1.0 / resolution_),
        chunk_size_(static_cast<int>(chunk_resolution * resolution_inv_)),
        default_value_(default_value),
        w_T_m_(origin_x, origin_y, origin_phi),
        m_T_w_(w_T_m_.inverse()),
        min_chunk_index_{0,0},
        max_chunk_index_{0,0},
        storage_(new storage_t)
    {
        storage_->insert({0,0}, chunk_t(chunk_size_, default_value_));
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
        fromIndex(getMaxIndex(),p);
        return p;
    }

    virtual inline muse_mcl_2d::Pose2D getOrigin() const
    {
        muse_mcl_2d::Transform2D origin_offset = w_T_m_;
        origin_offset.tx() += min_chunk_index_[0] * chunk_size_ * resolution_;
        origin_offset.ty() += min_chunk_index_[1] * chunk_size_ * resolution_;
        return origin_offset;
    }

    inline T& at(const std::size_t idx,
                 const std::size_t idy)
    {
        const index_t chunk_index = {min_chunk_index_[0] + static_cast<int>(idx) / chunk_size_,
                                     min_chunk_index_[1] + static_cast<int>(idy) / chunk_size_};
        const index_t local_chunk_index = toLocalChunkIndex({static_cast<int>(idx), static_cast<int>(idy)});

        if(chunk_index[0] > max_chunk_index_[0] ||
                chunk_index[1] > max_chunk_index_[1]) {
            throw std::runtime_error("Within current extent is allowed!");
        }

        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
            updateChunkIndices(chunk_index);
        }
        return chunk->at(local_chunk_index);
    }

    inline const T& at(const std::size_t idx,
                       const std::size_t idy) const
    {
        const index_t chunk_index = {min_chunk_index_[0] + idx / chunk_size_,
                                     min_chunk_index_[1] + idy / chunk_size_};
        const index_t local_chunk_index = toLocalChunkIndex({idx, idy});

        if(chunk_index[0] > max_chunk_index_[0] ||
                chunk_index[1] > max_chunk_index_[1]) {
            throw std::runtime_error("Within current extent is allowed!");
        }

        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
            updateChunkIndices(chunk_index);
        }
        return chunk->at(local_chunk_index);
    }

    inline T& at(const muse_mcl_2d::Point2D &point)
    {
        const index_t index = toIndex(point);
        const index_t chunk_index = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
            updateChunkIndices(chunk_index);
        }
        return chunk->at(local_chunk_index);
    }

    inline const T& at(const muse_mcl_2d::Point2D &point) const
    {
        const index_t index = toIndex(point);
        const index_t chunk_index = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
            updateChunkIndices(chunk_index);
        }
        return chunk->at(local_chunk_index);
    }

    inline double getResolution() const
    {
        return resolution_;
    }

    inline std::size_t getHeight() const
    {
        return (max_chunk_index_[1] - min_chunk_index_[1] + 1) * chunk_size_;
    }

    inline std::size_t getWidth() const
    {
        return (max_chunk_index_[0] - min_chunk_index_[0] + 1) * chunk_size_;
    }

    inline index_t getMaxIndex() const
    {
        return {(max_chunk_index_[0] - min_chunk_index_[0] + 1) * chunk_size_ - 1,
                (max_chunk_index_[1] - min_chunk_index_[1] + 1) * chunk_size_ - 1};
    }


protected:
    const double                      resolution_;
    const double                      resolution_inv_;
    const int                         chunk_size_;
    const T                           default_value_;
    muse_mcl_2d::Transform2D          w_T_m_;
    muse_mcl_2d::Transform2D          m_T_w_;


    mutable index_t                    min_chunk_index_;
    mutable index_t                    max_chunk_index_;
    mutable std::shared_ptr<storage_t> storage_;

    inline void updateChunkIndices(const index_t &chunk_index) const
    {
        min_chunk_index_[0] = std::min(min_chunk_index_[0], chunk_index[0]);
        min_chunk_index_[1] = std::min(min_chunk_index_[1], chunk_index[1]);
        max_chunk_index_[0] = std::max(max_chunk_index_[0], chunk_index[0]);
        max_chunk_index_[1] = std::max(max_chunk_index_[1], chunk_index[1]);
    }

    inline index_t toChunkIndex(const index_t &index) const
    {
        return {index[0] / chunk_size_,
                index[1] / chunk_size_};
    }

    inline index_t toLocalChunkIndex(const index_t &index) const
    {
        return {index[0] % chunk_size_,
                index[1] % chunk_size_};
    }

    inline index_t toIndex(const muse_mcl_2d::Point2D &p_w) const
    {
        const muse_mcl_2d::Point2D p_m = m_T_w_ * p_w;
        return {static_cast<int>(p_m.x() * resolution_inv_),
                static_cast<int>(p_m.y() * resolution_inv_)};
    }



    inline void fromIndex(const index_t &i,
                          muse_mcl_2d::Point2D &p_w) const
    {
        p_w = w_T_m_ * muse_mcl_2d::Point2D(i[0] * resolution_ - min_chunk_index_[0] * chunk_size_,
                                            i[1] * resolution_ - min_chunk_index_[1] * chunk_size_);
    }

    };
}
}



#endif // GRIDMAP_HPP
