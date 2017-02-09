#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>

#include <muse_amcl/data_types/map.hpp>

#include "bresenham.hpp"

namespace muse_amcl {
namespace maps {
template<typename T>
class GridMap : public Map
{
public:
    using Ptr = std::shared_ptr<GridMap>;
    using LineIterator = Bresenham<T const>;
    using Index = std::array<int, 2>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::size_t height,
            const std::size_t width,
            const std::string frame) :
        Map(frame),
        resolution_(resolution),
        height_(height),
        width_(width),
        max_index_({(int)(width)-1,(int)(height)-1}),
        origin_x_(origin_x),
        origin_y_(origin_y),
        origin_phi_(origin_phi),
        cos_phi_(cos(origin_phi)),
        sin_phi_(sin(origin_phi)),
        tx_(origin_x_),
        ty_(origin_y_)
    {
        if(origin_phi_ != 0.0) {
            tx_ =  cos_phi_ * origin_x +
                   sin_phi_ * origin_y;
            ty_ = -sin_phi_ * origin_x +
                   cos_phi_ * origin_y;
        }
    }

    virtual inline math::Point getMin() const override
    {
        math::Point p;
        fromIndex({0,0},p);
        return p;
    }

    virtual inline math::Point getMax() const override
    {
        math::Point p;
        fromIndex({(int)width_-1,(int)height_-1},p);
        return p;
    }

    virtual inline math::Pose getOrigin() const
    {
        math::Pose::Vector3d origin(origin_x_, origin_y_, origin_phi_);
        return math::Pose(origin);
    }

    inline bool toIndex(const math::Point &p,
                        Index &i) const
    {
        double _x = p.x();
        double _y = p.y();
        double x = _x;
        double y = _y;

        if(origin_phi_ != 0.0) {
            x =  cos_phi_ * _x +
                 sin_phi_ * _y;
            y = -sin_phi_ * _x +
                 cos_phi_ * _y;
        }

        i[0] = (x - tx_) / resolution_;
        i[1] = (y - ty_) / resolution_;

        return i[0] >= 0 && i[0] <= max_index_[0] ||
               i[1] >= 0 && i[1] <= max_index_[1];
    }

    inline void fromIndex(const Index &i,
                          math::Point &p) const
    {
        double &_x = p.x();
        double &_y = p.y();
        _x = i[0] * resolution_;
        _y = i[1] * resolution_;
        if(origin_phi_ != 0.0)  {
            double x = cos_phi_ * _x -
                    sin_phi_ * _y;
            double y = sin_phi_ * _x +
                    cos_phi_ * _y;
            _x = x;
            _y = y;
        }
        _x += origin_x_;
        _y += origin_y_;
    }

    inline T& at(const std::size_t idx,
                 const std::size_t idy)
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline const T& at(const std::size_t idx,
                       const std::size_t idy) const
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline T& at(const math::Point &point)
    {
        Index i;
        toIndex(point, i);
        if(invalid(i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    inline const T& at(const math::Point &point) const
    {
        Index i;
        toIndex(point, i);
        if(invalid(i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i);
    }

    inline LineIterator getLineIterator(const Index &start,
                                        const Index &_end) const
    {
        return LineIterator(cap(start), cap(_end), width_, data_ptr_);
    }

    inline LineIterator getLineIterator(const math::Point &start,
                                        const math::Point &end) const
    {
        Index start_index;
        Index end_index;
        toIndex(start, start_index);
        toIndex(end, end_index);
        return LineIterator(cap(start_index), cap(end_index), width_, data_ptr_);
    }

    inline double getResolution() const
    {
        return resolution_;
    }

    inline std::size_t getHeight() const
    {
        return height_;
    }

    inline std::size_t getWidth() const
    {
        return width_;
    }

    inline std::size_t getMaxIndex() const
    {
        return max_index_;
    }


protected:
    const double      resolution_;
    const std::size_t height_;
    const std::size_t width_;
    const Index       max_index_;

    std::vector<T>  data_;
    T*              data_ptr_;

    double      origin_x_;
    double      origin_y_;
    double      origin_phi_;

    double      cos_phi_;
    double      sin_phi_;
    double      tx_;
    double      ty_;

    inline Index cap(const Index &_i) const
    {
        return {(_i[0] < 0 ? 0 : (_i[0] > max_index_[0] ? max_index_[0] : _i[0])),
                (_i[1] < 0 ? 0 : (_i[1] > max_index_[1] ? max_index_[1] : _i[1]))};
    }

    inline bool invalid(const Index &_i) const
    {
        return _i[0] < 0 || _i[1] < 0 ||
               _i[0] > max_index_[0] || _i[1] > max_index_[1];
    }



    };

}
}
#endif /* GRIDMAP_HPP */
