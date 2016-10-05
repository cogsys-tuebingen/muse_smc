#pragma once

#include "map.hpp"

#include <vector>
#include <cmath>

namespace muse {
namespace maps {
template<typename T>
class GridMap : Map
{
public:
    typedef std::shared_ptr<GridMap> Ptr;

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

    inline bool toIndex(const double _x, const double _y,
                        std::size_t &_idx, std::size_t &_idy)
    {
        double x = _x;
        double y = _y;

        if(origin_phi != 0.0) {
            x =  cos_phi * _x +
                 sin_phi * _y;
            y = -sin_phi * _x +
                 cos_phi * _y;
        }

        _idx = (x - tx) / resolution;
        _idy = (y - ty) / resolution;

        return x < 0.0 || y < 0.0;
    }

    inline void fromIndex(const std::size_t _idx, const std::size_t _idy,
                          double &_x, double &_y)
    {
        _x = _idx * resolution;
        _y = _idy * resolution;
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

    const double      resolution;
    const std::size_t height;
    const std::size_t width;

protected:
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
