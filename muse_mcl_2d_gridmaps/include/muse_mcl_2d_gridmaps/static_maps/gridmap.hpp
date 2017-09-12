#ifndef GRIDMAP_HPP
#define GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>

#include <muse_mcl_2d/map/map_2d.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/bresenham.hpp>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
template<typename T>
class GridMap : public muse_mcl_2d::Map2D
{
public:
    using Ptr              = std::shared_ptr<GridMap<T>>;
    using line_iterator_t  = algorithms::Bresenham<T const>;
    using index_t          = std::array<int, 2>;

    GridMap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::size_t height,
            const std::size_t width,
            const T &default_value,
            const std::string frame) :
        Map2D(frame),
        resolution_(resolution),
        resolution_inv_(1.0 / resolution),
        height_(height),
        width_(width),
        max_index_({(int)(width)-1,(int)(height)-1}),
        w_T_m_(origin_x, origin_y, origin_phi),
        m_T_w_(w_T_m_.inverse())
    {
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
        return w_T_m_;
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
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    inline const T& at(const muse_mcl_2d::Point2D &point) const
    {
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i);
    }

    inline line_iterator_t getLineIterator(const index_t &start,
                                           const index_t &end) const
    {
        return line_iterator_t(start, end, width_, data_ptr_);
    }

    inline line_iterator_t getLineIterator(const muse_mcl_2d::Point2D &start,
                                           const muse_mcl_2d::Point2D &end) const
    {
        index_t start_index;
        index_t end_index;
        toIndex(start, start_index);
        toIndex(end, end_index);
        return line_iterator_t(start_index,
                            end_index,
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

    inline index_t getMaxIndex() const
    {
        return max_index_;
    }


protected:
    const double                      resolution_;
    const double                      resolution_inv_;
    const std::size_t                 height_;
    const std::size_t                 width_;
    const index_t                     max_index_;
    const muse_mcl_2d::Transform2D    w_T_m_;
    const muse_mcl_2d::Transform2D    m_T_w_;

    std::vector<T>                    data_;
    T*                                data_ptr_;

    inline bool invalid(const index_t &_i) const
    {
        return _i[0] < 0 ||
               _i[1] < 0 ||
               _i[0] > max_index_[0] ||
               _i[1] > max_index_[1];
    }

    inline bool toIndex(const muse_mcl_2d::Point2D &p_w,
                        index_t &i) const
    {
        const muse_mcl_2d::Point2D p_m = m_T_w_ * p_w;

        i[0] = static_cast<int>(p_m.x() * resolution_inv_ + 0.5);
        i[1] = static_cast<int>(p_m.y() * resolution_inv_ + 0.5);

        return (i[0] >= 0 && i[0] <= max_index_[0]) ||
               (i[1] >= 0 && i[1] <= max_index_[1]);
    }

    inline void fromIndex(const index_t &i,
                          muse_mcl_2d::Point2D &p_w) const
    {
        p_w = w_T_m_ * muse_mcl_2d::Point2D(i[0] * resolution_,
                                            i[1] * resolution_);
    }

    inline void fromIndex(const line_iterator_t &it,
                          muse_mcl_2d::Point2D &p_w) const
    {
        p_w = w_T_m_ * muse_mcl_2d::Point2D(it.x() * resolution_,
                                            it.y() * resolution_);
    }

    };

}
}
#endif /* GRIDMAP_HPP */
