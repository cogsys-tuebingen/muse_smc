#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include <array>

namespace muse_mcl_2d_gridmaps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    using index_t = std::array<int, 2>;
    using size_t  = std::array<int, 2>;

    inline explicit Bresenham(const index_t &start,
                              const index_t &end,
                              const size_t   size,
                              T *            data) :
        done_(false),
        data_(data),
        size_(size),
        step_(size[0]),
        start_(start),
        end_(end),
        steep_(std::abs(end[1] - start[1]) > std::abs(end[0] - start[0])),
        error_(0)
    {
        if(steep_) {
            std::swap(start_[0], start_[1]);
            std::swap(end_[0],   end_[1]);
            std::swap(size_[0],  size_[1]);
        }

        delta_x_ = std::abs(end_[0] - start_[0]);
        delta_y_ = std::abs(end_[1] - start_[1]);
        delta_error_ = delta_y_;
        index_ = start_;

        data_pos_ = pos();

        step_x_ = start_[0] < end_[0] ? 1 : -1;
        step_y_ = start_[1] < end_[1] ? 1 : -1;

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
        if(done() || invalid())
            return *this;

        index_[0] += step_x_;

        error_ += delta_error_;
        if(2 * error_ >= delta_x_) {
            index_[1] += step_y_;
            error_ -= delta_x_;
        }
        data_pos_ = pos();

        return *this;
    }

    inline bool done() const
    {
        return index_[0] == end_[0] && index_[1] == end_[1];
    }

    inline bool invalid() const
    {
        return index_[0] < 0 ||
               index_[1] < 0 ||
               index_[0] >= size_[0] ||
               index_[1] >= size_[1];
    }

    inline T& operator *() const
    {
        return data_[data_pos_];
    }


private:
    inline int pos() const
    {
        if(steep_)
            return index_[0] * step_ + index_[1];
        return index_[1] * step_ + index_[0];
    }

    bool         done_;
    T           *data_;
    int          data_pos_;
    size_t       size_;
    int          step_;

    index_t      start_;
    index_t      end_;
    index_t      index_;

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
