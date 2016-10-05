#pragma once

#include "map.hpp"

#include <array>

namespace muse {
namespace maps {
template<typename T>
class GridMapLineIterator
{
    const T *data;
    const std::size_t step;

    using parent = std::iterator<std::random_access_iterator_tag, T>;
    using iterator = typename parent::iterator;
    using reference = typename parent::reference;

public:
    typedef std::array<std::size_t, 2> Index;
    typedef std::array<int, 2> Step;

    inline explicit GridMapLineIterator(const Index &_start,
                                        const Index &_end,
                                        const std::size_t _step,
                                        T *_data) :
        data(_data),
        step(_step),
        start(_start),
        end(_end),
        steep(std::abs(_end[0] - _start[0]) > std::abs(_end[1] - start[1]))
    {
        if(steep) {
            std::swap(start[0], start[1]);
            std::swap(end[0], end[1]);
        }
    }

    inline std::size_t x() const
    {
        return index[0];
    }

    inline std::size_t y() const
    {
        return index[1];
    }

    iterator& operator++()
    {





        ++data;
        return *this;
    }

    bool operator ==(const GridMapLineIterator &_other) const
    {
        return data == _other.data;
    }

    bool operator !=(const GridMapLineIterator &_other) const
    {
        return !(*this == _other);
    }

    reference operator *() const
    {
        return *data;
    }

private:
    Index start;
    Index end;
    Index index;

    bool  steep;
    int   error;
    int   delta_error;
    int   step_x;
    int   step_y;


};

}
}
