#pragma once

#include "map.hpp"

#include <array>

namespace muse {
namespace maps {
template<typename T>
class GridMapLineIterator
{
public:
    typedef std::array<int, 2> Index;

    inline explicit GridMapLineIterator(const Index      &_start,
                                        const Index      &_end,
                                        const std::size_t _step,
                                        T *               _data) :
        data(_data),
        step(_step),
        start(_start),
        end(_end),
        steep(std::abs(_end[0] - _start[0]) <= std::abs(_end[1] - _start[1])),
        error(0)
    {
        if(steep) {
            std::swap(start[0], start[1]);
            std::swap(end[0], end[1]);
        }

        delta_x = std::abs(end[0] - start[0]);
        delta_y = std::abs(end[1] - start[1]);
        delta_error = delta_y;
        index = start;
        data += steep ? (index[0] * step + index[1]) : (index[1] * step + index[0]);

        step_x = start[0] < end[0] ? 1 : -1;
        step_y = start[1] < end[1] ? 1 : -1;

    }

    inline std::size_t x() const
    {
        return steep ? index[1] : index[0];
    }

    inline std::size_t y() const
    {
        return steep ? index[0] : index[1];
    }

    inline GridMapLineIterator& operator++()
    {
        if(done())
            return *this;

        index[0] += step_x;
        data += steep ? step * step_y : step_x;

        error += delta_error;
        if(2 * error >= delta_x) {
            index[1] += step_y;
            error -= delta_x;
            data += steep ? step_x : step * step_y;
        }

        return *this;
    }

    inline bool done()
    {
        return index[0] == end[0] && index[1] == end[1];
    }

    inline T& operator *() const
    {
        return *data;
    }

private:
    T           *data;
    std::size_t  step;

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
