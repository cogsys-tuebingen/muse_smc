#pragma once

#include "map.hpp"
#include "bresenham.hpp"

#include <array>
#include <vector>
#include <cmath>

namespace muse {
namespace maps {
template<typename T>
class GridMap : public Map
{
public:
    typedef std::shared_ptr<GridMap>    Ptr;
    typedef Bresenham<const T>          LineIterator;
    typedef std::array<int, 2>          Index;
    typedef std::array<double, 2>       Position;

    GridMap(const double _origin_x,
            const double _origin_y,
            const double _origin_phi,
            const double _resolution,
            const std::size_t _height,
            const std::size_t _width,
            const std::string _frame) :
        Map(_frame),
        resolution(_resolution),
        height(_height),
        width(_width),
        max_index({(int)(_width)-1,(int)(_height)-1}),
        origin_x(_origin_x),
        origin_y(_origin_y),
        origin_phi(_origin_phi),
        cos_phi(cos(_origin_phi)),
        sin_phi(sin(_origin_phi)),
        tx(origin_x),
        ty(origin_y)
    {
        if(origin_phi != 0.0) {
            tx =  cos_phi * _origin_x +
                    sin_phi * _origin_y;
            ty = -sin_phi * _origin_x +
                    cos_phi * _origin_y;
        }
    }

    inline bool toIndex(const Position &_p,
                        Index &_i)
    {
        double _x = _p[0];
        double _y = _p[1];
        double x = _x;
        double y = _y;

        if(origin_phi != 0.0) {
            x =  cos_phi * _x +
                    sin_phi * _y;
            y = -sin_phi * _x +
                    cos_phi * _y;
        }

        _i[0] = (x - tx) / resolution;
        _i[1] = (y - ty) / resolution;

        return x < 0.0 || y < 0.0;
    }

    inline void fromIndex(const Index &_i,
                          Position &_p)
    {
        double &_x = _p[0];
        double &_y = _p[1];
        _x = _i[0] * resolution;
        _y = _i[1] * resolution;
        if(origin_phi != 0.0)  {
            double x = cos_phi * _x -
                    sin_phi * _y;
            double y = sin_phi * _x +
                    cos_phi * _y;
            _x = x;
            _y = y;
        }
        _x += origin_x;
        _y += origin_y;
    }

    inline T& at(const std::size_t _idx,
                 const std::size_t _idy)
    {
        return data_ptr[width * _idy + _idx];
    }

    inline const T& at(const std::size_t _idx,
                       const std::size_t _idy) const
    {
        return data_ptr[width * _idy + _idx];
    }

    LineIterator getLineIterator(const Index &_start,
                                 const Index &_end) const
    {
        return LineIterator(cap(_start), cap(_end), width, data_ptr);
    }

    LineIterator getLineIterator(const Position &_start,
                                 const Position &_end) const
    {
        Index start;
        Index end;
        toIndex(_start, start);
        toIndex(_end, end);
        return LineIterator(cap(start), cap(end), width, data_ptr);
    }
    const double      resolution;
    const std::size_t height;
    const std::size_t width;
    const Index       max_index;

protected:
    inline Index cap(const Index &_i)
    {
        return {(_i[0] < 0 ? 0 : (_i[0] > max_index[0] ? max_index[0] : _i[0])),
                (_i[1] < 0 ? 0 : (_i[1] > max_index[1] ? max_index[1] : _i[1]))};
    }

    inline bool invalid(const Index &_i)
    {
        return _i[0] < 0 || _i[1] < 0 ||
               _i[0] > max_index[0] || _i[1] > max_index[1];
    }

    std::vector<T> data;
    T*             data_ptr;

    double      origin_x;
    double      origin_y;
    double      origin_phi;

    double      cos_phi;
    double      sin_phi;
    double      tx;
    double      ty;


    };

}
}
