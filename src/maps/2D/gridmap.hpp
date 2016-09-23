#pragma once

#include <tf/tf.h>
#include <memory>

namespace muse {
namespace maps {
template<typename T>
class GridMap
{
public:
    typedef std::shared_ptr<GridMap> Ptr;

    GridMap(const double _origin_x,
            const double _origin_y,
            const double _origin_phi,
            const double _resolution,
            const std::size_t _height,
            const std::size_t _width) :
        origin_x(_origin_x),
        origin_y(_origin_y),
        origin_phi(_origin_phi),
        cos_phi(cos(_origin_phi)),
        sin_phi(sin(_origin_phi)),
        tx(origin_x),
        ty(origin_y),
        resolution(_resolution),
        height(_height),
        width(_width)
    {
        if(origin_phi != 0.0) {
            tx =  cos_phi * _origin_x +
                  sin_phi * _origin_y;
            ty = -sin_phi * _origin_x +
                  cos_phi * _origin_y;
        }
    }

    GridMap(const tf::Pose &_origin,
            const double _resolution,
            const std::size_t _height,
            const std::size_t _width) :
        origin_x(_origin.getOrigin().x()),
        origin_y(_origin.getOrigin().y()),
        origin_phi(tf::getYaw(_origin.getRotation())),
        cos_phi(cos(origin_phi)),
        sin_phi(sin(origin_phi)),
        tx(origin_x),
        ty(origin_y),
        resolution(_resolution),
        height(_height),
        width(_width)
    {
        if(origin_phi != 0.0) {
            tx =  cos_phi * _origin.getOrigin().x() +
                  sin_phi * _origin.getOrigin().y();
            ty = -sin_phi * _origin.getOrigin().x() +
                  cos_phi * _origin.getOrigin().y();
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

    inline std::size_t getHeight() const
    {
        return height;
    }

    inline std::size_t getWidth() const
    {
        return width;
    }


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

    double      resolution;
    std::size_t height;
    std::size_t width;

};

}
}
