#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include <array>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>

#include <muse_mcl_2d_gridmaps/dynamic_maps/chunk.hpp>

namespace cis = cslibs_indexed_storage;

namespace muse_mcl_2d_gridmaps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    using index_t   = std::array<int, 2>;
    using chunk_t   = dynamic_maps::Chunk<T>;
    using storage_t = cis::Storage<chunk_t, index_t, cis::backend::kdtree::KDTree>;

    inline explicit Bresenham(const index_t                    &start,
                              const index_t                    &end,
                              const int                         chunk_size,
                              const index_t                    &min_chunk_index,
                              const T                          &default_value,
                              const std::shared_ptr<storage_t> &storage) :
        done_(false),
        storage_(storage),
        chunk_size_(chunk_size),
        start_(start),
        end_(end),
        min_chunk_index_(min_chunk_index),
        default_value_(default_value),
        steep_(std::abs(end[1] - start[1]) > std::abs(end[0] - start[0])),
        error_(0)
    {
        if(steep_) {
            std::swap(start_[0], start_[1]);
            std::swap(end_[0],   end_[1]);
        }

        delta_x_     = std::abs(end_[0] - start_[0]);
        delta_y_     = std::abs(end_[1] - start_[1]);
        delta_error_ = delta_y_;
        index_       = start_;

        step_x_      = start_[0] < end_[0] ? 1 : -1;
        step_y_      = start_[1] < end_[1] ? 1 : -1;

        updateChunk();
        updateLocalIndex();
    }

    inline int x() const
    {
        return steep_ ? index_[1] : index_[0];
    }

    inline int y() const
    {
        return steep_ ? index_[0] : index_[1];
    }

    inline Bresenham& operator++()
    {
        if(localIndexInvalid()) {
            updateChunk();
            updateLocalIndex();
        }
        std::cout << chunk_index_[0] << " " << chunk_index_[1] << std::endl;
        std::cout << local_index_[0] << " " << local_index_[1] << std::endl;

        if(done())
            return *this;

        index_[0]       += step_x_;
        local_index_[0] += step_x_;

        error_ += delta_error_;
        if(2 * error_ >= delta_x_) {
            index_[1]       += step_y_;
            local_index_[1] += step_y_;
            error_ -= delta_x_;
        }

        return *this;
    }

    inline bool done() const
    {
        return index_[0] == end_[0] && index_[1] == end_[1];
    }

    inline T& operator *() const
    {
        if(steep_) {
            return active_chunk_->at(local_index_[1], local_index_[0]);
        } else {
            return active_chunk_->at(local_index_[0], local_index_[1]);
        }
    }


private:
    inline void updateChunk()
    {
        if(steep_) {
            chunk_index_[0] = min_chunk_index_[0] + static_cast<int>(index_[1]) / chunk_size_;
            chunk_index_[1] = min_chunk_index_[1] + static_cast<int>(index_[0]) / chunk_size_;
        } else {
            chunk_index_[0] = min_chunk_index_[0] + static_cast<int>(index_[0]) / chunk_size_;
            chunk_index_[1] = min_chunk_index_[1] + static_cast<int>(index_[1]) / chunk_size_;
        }
        active_chunk_ = storage_->get(chunk_index_);
        if(active_chunk_ == nullptr) {
            active_chunk_ = &(storage_->insert(chunk_index_, chunk_t(chunk_size_, default_value_)));
        }
    }

    inline void updateLocalIndex()
    {
        local_index_[0] = index_[0] % chunk_size_;
        local_index_[1] = index_[1] % chunk_size_;
    }

    inline bool localIndexInvalid()
    {
        return local_index_[0] < 0 || local_index_[0] >= chunk_size_ ||
               local_index_[1] < 0 || local_index_[1] >= chunk_size_;
    }


    bool                        done_;
    std::shared_ptr<storage_t>  storage_;
    chunk_t                    *active_chunk_;
    int                         chunk_size_;

    index_t      start_;
    index_t      end_;
    index_t      index_;

    index_t      chunk_index_;
    index_t      local_index_;
    index_t      min_chunk_index_;
    T            default_value_;

    bool         steep_;
    int          error_;
    int          delta_x_;
    int          delta_y_;
    int          delta_error_;
    int          step_x_;
    int          step_y_;


};

}
}

#endif /* BRESENHAM_HPP */
