#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>

#include <muse_mcl_2d/map/map_2d.hpp>
#include <muse_mcl_2d_gridmaps/algorithms/bresenham.hpp>

namespace muse_mcl_2d_gridmaps {
namespace maps {
template<typename T>
class GridMap : public muse_mcl_2d::Map2D
{
public:
    using Ptr           = std::shared_ptr<GridMap>;
    using LineIterator  = algorithms::Bresenham<T const>;
    using Index         = std::array<int, 2>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::size_t height,
            const std::size_t width,
            const std::string frame) :
        Map2D(frame),
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
            tx_ =  -cos_phi_ * origin_x
                   -sin_phi_ * origin_y;
            ty_ =   sin_phi_ * origin_x
                   -cos_phi_ * origin_y;
        } else {
            tx_ = -origin_x;
            ty_ = -origin_y;
        }
    }

    virtual inline muse_mcl_2d::Point2D getMin() const override
    {
        muse_mcl_2d::Point2D p;
        fromIndex({0,0},p);
        return p;
    }

    virtual inline muse_mcl_2d::Point2D getMax() const override
    {
        muse_mcl_2d::Point2D p;
        fromIndex({(int)width_-1,(int)height_-1},p);
        return p;
    }

    virtual inline muse_mcl_2d::Pose2D getOrigin() const
    {
        return muse_mcl_2d::Pose2D(origin_x_, origin_y_, origin_phi_);
    }

    inline bool toIndex(const muse_mcl_2d::Point2D &p,
                        Index &i) const
    {
        double _x = p.x();
        double _y = p.y();
        double x = _x;
        double y = _y;

        if(origin_phi_ != 0.0) {
            x =   cos_phi_ * _x +
                  sin_phi_ * _y;
            y =  -sin_phi_ * _x +
                  cos_phi_ * _y;
        }

        i[0] = std::floor((x + tx_) / resolution_ + 0.5);
        i[1] = std::floor((y + ty_) / resolution_ + 0.5);

        return (i[0] >= 0 && i[0] <= max_index_[0]) ||
               (i[1] >= 0 && i[1] <= max_index_[1]);
    }

    inline void fromIndex(const Index &i,
                          muse_mcl_2d::Point2D &p) const
    {
        double &_x = p.x();
        double &_y = p.y();
        _x = i[0] * resolution_;
        _y = i[1] * resolution_;
        if(origin_phi_ != 0.0)  {
            const double x =  cos_phi_ * _x -
                              sin_phi_ * _y;
            const double y =  sin_phi_ * _x +
                              cos_phi_ * _y;

            _x = x;
            _y = y;
        }
        _x += origin_x_;
        _y += origin_y_;
    }

    inline void fromIndex(const LineIterator &it,
                          muse_mcl_2d::Point2D &p) const
    {
        double &_x = p.x();
        double &_y = p.y();
        _x = it.x() * resolution_;
        _y = it.y() * resolution_;
        if(origin_phi_ != 0.0)  {
            const double x =  cos_phi_ * _x -
                              sin_phi_ * _y;
            const double y =  sin_phi_ * _x +
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

    inline T& at(const muse_mcl_2d::Point2D &point)
    {
        Index i;
        toIndex(point, i);
        if(invalid(i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    inline const T& at(const muse_mcl_2d::Point2D &point) const
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
        return LineIterator(start, _end, width_, data_ptr_);
    }

    inline LineIterator getLineIterator(const muse_mcl_2d::Point2D &start,
                                        const muse_mcl_2d::Point2D &end) const
    {
        Index start_index;
        Index end_index;
        toIndex(start, start_index);
        toIndex(end, end_index);
        return LineIterator(start_index, end_index,
                            {static_cast<int>(width_), static_cast<int>(height_)},
                            data_ptr_);
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
    double      tx_inv_;
    double      ty_inv_;

    /**
      * @todo: implement capping by inesection with bounding box implementation
      *        maybe utilize boost geometry.
      */

    /**
     * @brief invalid
     * @param _i
     * @return
     */

    inline bool invalid(const Index &_i) const
    {
        return _i[0] < 0 ||
               _i[1] < 0 ||
               _i[0] > max_index_[0] ||
               _i[1] > max_index_[1];
    }



    };

}
}
#endif /* GRIDMAP_HPP */
