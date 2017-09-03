#ifndef BRESENHAM_HPP
#define BRESENHAM_HPP

#include <array>

namespace muse_mcl_2d_gridmaps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    typedef std::array<int, 2> Index;
    typedef std::array<int, 2> Size;

    inline explicit Bresenham(const Index      &_start,
                              const Index      &_end,
                              const Size        _size,
                              T *               _data) :
        done_(false),
        data(_data),
        size(_size),
        step(_size[0]),
        start(_start),
        end(_end),
        steep(std::abs(_end[1] - _start[1]) > std::abs(_end[0] - _start[0])),
        error(0)
    {
        if(steep) {
            std::swap(start[0], start[1]);
            std::swap(end[0],   end[1]);
            std::swap(size[0],  size[1]);
        }

        delta_x = std::abs(end[0] - start[0]);
        delta_y = std::abs(end[1] - start[1]);
        delta_error = delta_y;
        index = start;

        data_pos = pos();

        step_x = start[0] < end[0] ? 1 : -1;
        step_y = start[1] < end[1] ? 1 : -1;

    }

    inline int x() const
    {
        return steep ? index[1] : index[0];
        }

        inline int y() const
        {
        return steep ? index[0] : index[1];
    }

    inline Bresenham& operator++()
    {
        if(done() || invalid())
            return *this;

        index[0] += step_x;

        error += delta_error;
        if(2 * error >= delta_x) {
            index[1] += step_y;
            error -= delta_x;
        }
        data_pos = pos();

        return *this;
    }

    inline bool done() const
    {
        return index[0] == end[0] && index[1] == end[1];
    }

    inline bool invalid() const
    {
        return index[0] < 0 ||
               index[1] < 0 ||
               index[0] >= size[0] ||
               index[1] >= size[1];
    }

    inline T operator *() const
    {
        return data[data_pos];
    }

private:
    inline int pos() const
    {
        if(steep)
            return index[0] * step + index[1];
        return index[1] * step + index[0];
    }

    bool         done_;
    T           *data;
    int          data_pos;
    Size         size;
    int          step;

    Index        start;
    Index        end;
    Index        index;

    bool         steep;
    int          error;
    int          delta_x;
    int          delta_y;
    int          delta_error;
    int          step_x;
    int          step_y;


};

}
}

#endif /* BRESENHAM_HPP */
