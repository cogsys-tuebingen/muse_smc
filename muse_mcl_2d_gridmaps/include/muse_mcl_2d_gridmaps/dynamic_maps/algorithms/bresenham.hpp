#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include <array>

#include <muse_smc/utility/delegate.hpp>

#include <cslibs_math/common/div.hpp>
#include <cslibs_math/common/mod.hpp>
#include <cslibs_math/common/array.hpp>

#include <muse_mcl_2d_gridmaps/dynamic_maps/chunk.hpp>

#include <iostream>

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    using Ptr           = std::shared_ptr<Bresenham>;
    using index_t       = std::array<int, 2>;
    using chunk_t       = dynamic_maps::Chunk<T>;
    using get_chunk_t   = delegate<chunk_t*(const index_t&)>;

    inline explicit Bresenham(const index_t     &start,
                              const index_t     &end,
                              const int          chunk_size,
                              const T           &default_value,
                              const get_chunk_t &get_chunk) :
        done_(false),
        get_chunk_(get_chunk),
        active_chunk_(nullptr),
        chunk_size_(chunk_size),
        start_(start),
        end_(end),
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

        update();
    }

    inline virtual ~Bresenham()
    {
        if(active_chunk_) {
            active_chunk_->unlock();
        }
    }

    inline int x() const
    {
        return (steep_ ? index_[1] : index_[0]);
    }

    inline int y() const
    {
        return (steep_ ? index_[0] : index_[1]);
    }

    inline int lx() const
    {
        return (steep_ ? local_index_[1] : local_index_[0]);
    }

    inline int ly() const
    {
        return (steep_ ? local_index_[0] : local_index_[1]);
    }

    inline Bresenham& operator++()
    {
        if(done()) {
            if(active_chunk_) {
                active_chunk_->unlock();
                active_chunk_ = nullptr;
            }
            return *this;
        }

        index_[0]       += step_x_;
        local_index_[0] += step_x_;

        error_ += delta_error_;
        if(2 * error_ >= delta_x_) {
            index_[1]       += step_y_;
            local_index_[1] += step_y_;
            error_          -= delta_x_;
        }

        if(localIndexInvalid()) {
            update();
        }

        return *this;
    }

    inline bool done() const
    {
        return index_[0] == end_[0] && index_[1] == end_[1];
    }

    inline int length2() const
    {
        auto sq = [](const int d) { return d*d;};
        return sq(index_[0] - end_[0]) + sq(index_[1] - end_[1]);
    }

    inline T& operator *() const
    {
        assert(active_chunk_);
        return active_chunk_->at(lx(), ly());
    }

private:
    inline void update()
    {
        if(active_chunk_) {
            active_chunk_->unlock();
        }

        chunk_index_[0] = cslibs_math::common::div(x(), chunk_size_);
        chunk_index_[1] = cslibs_math::common::div(y(), chunk_size_);
        local_index_[0] = cslibs_math::common::mod(index_[0], chunk_size_);
        local_index_[1] = cslibs_math::common::mod(index_[1], chunk_size_);

        active_chunk_ = get_chunk_(chunk_index_);
        active_chunk_->lock();

        std::cout << "new active chunk " << chunk_index_ << std::endl;
    }

    inline bool localIndexInvalid()
    {
        return local_index_[0] < 0 || local_index_[0] >= chunk_size_ ||
               local_index_[1] < 0 || local_index_[1] >= chunk_size_;
    }

    bool                        done_;
    get_chunk_t                 get_chunk_;
    chunk_t                    *active_chunk_;
    int                         chunk_size_;

    index_t      start_;
    index_t      end_;
    index_t      index_;

    index_t      chunk_index_;
    index_t      local_index_;
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
}

#endif /* BRESENHAM_HPP */
