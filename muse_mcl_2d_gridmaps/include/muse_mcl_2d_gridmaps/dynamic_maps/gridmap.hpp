#ifndef DYNAMIC_GRIDMAP_HPP
#define DYNAMIC_GRIDMAP_HPP

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
    using Ptr                   = std::shared_ptr<GridMap<T>>;
    using index_t               = std::array<int, 2>;
    using mutex_t               = std::mutex;
    using lock_t                = std::unique_lock<mutex_t>;
    using chunk_t               = Chunk<T>;
    using storage_t             = cis::Storage<chunk_t, index_t, cis::backend::kdtree::KDTree>;
    using line_iterator_t       = algorithms::Bresenham<T>;
    using const_line_iterator_t = algorithms::Bresenham<T const>;

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
        storage_(new storage_t),
        height_(chunk_size_),
        width_(chunk_size_)
    {
        storage_->insert({0,0}, chunk_t(chunk_size_, default_value_));
    }

    virtual inline muse_mcl_2d::math::Point2D getMin() const override
    {
        muse_mcl_2d::math::Point2D p;
        fromIndex({0,0},p);
        return p;
    }

    virtual inline muse_mcl_2d::math::Point2D getMax() const override
    {
        muse_mcl_2d::math::Point2D p;
        fromIndex(getMaxIndex(),p);
        return p;
    }

    virtual inline muse_mcl_2d::math::Pose2D getOrigin() const
    {
        muse_mcl_2d::math::Transform2D origin_offset = w_T_m_;
        origin_offset.tx() += min_chunk_index_[0] * chunk_size_ * resolution_;
        origin_offset.ty() += min_chunk_index_[1] * chunk_size_ * resolution_;
        return origin_offset;
    }

    inline T& at(const std::size_t idx,
                 const std::size_t idy)
    {
        if(invalid(idx, idy)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }

        const index_t chunk_index       = toChunkIndex(idx, idy);
        const index_t local_chunk_index = toLocalChunkIndex(idx,idy);
        lock_t l(storage_mutex_);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
        }
        return chunk->at(local_chunk_index);
    }

    virtual inline T at(const std::size_t idx,
                        const std::size_t idy) const
    {
        if(invalid(idx, idy)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }

        const index_t chunk_index       = toChunkIndex(idx, idy);
        const index_t local_chunk_index = toLocalChunkIndex(idx,idy);

        lock_t l(storage_mutex_);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            return default_value_;
        }
        return chunk->at(local_chunk_index);
    }

    virtual inline T& at(const muse_mcl_2d::math::Point2D &point)
    {
        const index_t index             = toIndex(point);
        const index_t chunk_index       = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);

        lock_t l(storage_mutex_);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
            updateChunkIndices(chunk_index);
        }

        return chunk->at(local_chunk_index);
    }

    virtual inline T at(const muse_mcl_2d::math::Point2D &point) const
    {
        const index_t index = toIndex(point);
        const index_t chunk_index = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);

        lock_t l(storage_mutex_);
        chunk_t *chunk = storage_->get(chunk_index);
        if(chunk == nullptr) {
            return default_value_;
        }
        return chunk->at(local_chunk_index);
    }

    inline line_iterator_t getLineIterator(const index_t &start,
                                           const index_t &end) const
    {
        /// do index capping
        if(invalid(start)) {
            throw std::runtime_error("[GridMap]: Start index is invalid!");
        }
        if(invalid(end)) {
            throw std::runtime_error("[GridMap]: End index is invalid!");
        }
        return line_iterator_t(start, end,
                               chunk_size_,
                               min_chunk_index_,
                               default_value_,
                               storage_);
    }

    inline typename line_iterator_t::Ptr getLineIterator(const muse_mcl_2d::math::Point2D &start,
                                                const muse_mcl_2d::math::Point2D &end) const
    {

        const index_t start_index = toIndex(start);
        const index_t end_index   = toIndex(end);

        const index_t start_chunk_index = toChunkIndex(start_index);
        const index_t end_chunk_index   = toChunkIndex(end_index);

        lock_t l(storage_mutex_);
        if(storage_->get(start_chunk_index) == nullptr) {
            storage_->insert(start_chunk_index, chunk_t(chunk_size_, default_value_));
            updateChunkIndices(start_chunk_index);
        }
        if(storage_->get(end_chunk_index) == nullptr) {
            storage_->insert(end_chunk_index, chunk_t(chunk_size_, default_value_));
            updateChunkIndices(end_chunk_index);
        }

        return typename line_iterator_t::Ptr(new line_iterator_t({start_index[0] - min_chunk_index_[0] * chunk_size_,
                                                                  start_index[1] - min_chunk_index_[1] * chunk_size_},
                                                                 {end_index[0] - min_chunk_index_[0] * chunk_size_,
                                                                  end_index[1] - min_chunk_index_[1] * chunk_size_},
                                                                  chunk_size_,
                                                                  min_chunk_index_,
                                                                  default_value_,
                                                                  storage_));
    }

    inline index_t getMinChunkIndex() const
    {
        return min_chunk_index_;
    }

    inline index_t getMaxChunkIndex() const
    {
        return max_chunk_index_;
    }

    inline chunk_t const * getChunk(const index_t &index)
    {
        return storage_->get(index);
    }

    inline double getResolution() const
    {
        return resolution_;
    }

    inline int getChunkSize() const
    {
        return chunk_size_;
    }

    inline std::size_t getHeight() const
    {
        return height_;
    }

    inline std::size_t getWidth() const
    {
        return width_;
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
    muse_mcl_2d::math::Transform2D    w_T_m_;
    muse_mcl_2d::math::Transform2D    m_T_w_;


    mutable index_t                    min_chunk_index_;
    mutable index_t                    max_chunk_index_;
    mutable mutex_t                    storage_mutex_;
    mutable std::shared_ptr<storage_t> storage_;
    mutable int                        height_;
    mutable int                        width_;

    inline bool invalid(const index_t &_i) const
    {
        return _i[0] < 0 ||
                _i[1] < 0 ||
                _i[0] >= width_ ||
                _i[1] >= height_;
    }

    inline bool invalid(const int idx, const int idy) const
    {
        return idx < 0 || idx >= width_ ||
                idy < 0 || idy >= height_;
    }

    inline void updateChunkIndices(const index_t &chunk_index) const
    {
        min_chunk_index_[0] = std::min(min_chunk_index_[0], chunk_index[0]);
        min_chunk_index_[1] = std::min(min_chunk_index_[1], chunk_index[1]);
        max_chunk_index_[0] = std::max(max_chunk_index_[0], chunk_index[0]);
        max_chunk_index_[1] = std::max(max_chunk_index_[1], chunk_index[1]);
        width_  = (max_chunk_index_[0] - min_chunk_index_[0] + 1) * chunk_size_;
        height_ = (max_chunk_index_[1] - min_chunk_index_[1] + 1) * chunk_size_;
    }

    inline index_t toChunkIndex(const int idx, const int idy) const
    {
        return  {min_chunk_index_[0] + static_cast<int>(idx) / chunk_size_,
                    min_chunk_index_[1] + static_cast<int>(idy) / chunk_size_};
    }

    inline index_t toLocalChunkIndex(const int idx, const int idy) const
    {
        return {idx % chunk_size_,
                    idy % chunk_size_};
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

    inline index_t toIndex(const muse_mcl_2d::math::Point2D &p_w) const
    {
        const muse_mcl_2d::math::Point2D p_m = m_T_w_ * p_w;
        return {static_cast<int>(p_m.x() * resolution_inv_),
                    static_cast<int>(p_m.y() * resolution_inv_)};
    }



    inline void fromIndex(const index_t &i,  muse_mcl_2d::math::Point2D &p_w) const
    {
        p_w = w_T_m_ * muse_mcl_2d::math::Point2D(i[0] * resolution_ - min_chunk_index_[0] * chunk_size_,
                i[1] * resolution_ - min_chunk_index_[1] * chunk_size_);
    }

};
}
}



#endif // DYNAMIC_GRIDMAP_HPP
