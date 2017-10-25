#ifndef STATIC_GRIDMAP_HPP
#define STATIC_GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>

#include <muse_mcl_2d/map/map_2d.hpp>
#include <cslibs_math_2d/types/pose.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/bresenham.hpp>
#include <muse_mcl_2d_gridmaps/static_maps/algorithms/bresenham_unsafe.hpp>

namespace muse_mcl_2d_gridmaps {
namespace static_maps {
template<typename T>
class Gridmap : public muse_mcl_2d::Map2D
{
public:
    using Ptr                    = std::shared_ptr<Gridmap<T>>;
    using const_line_iterator_t  = algorithms::Bresenham<T const>;
    using index_t                = std::array<int, 2>;
    using pose_t                 = cslibs_math_2d::Pose2d;

    Gridmap(const pose_t &origin,
            const double resolution,
            const std::size_t height,
            const std::size_t width,
            const T &default_value,
            const std::string &frame) :
        Map2D(frame),
        resolution_(resolution),
        resolution_inv_(1.0 / resolution),
        height_(height),
        width_(width),
        max_index_({(int)(width)-1,(int)(height)-1}),
        w_T_m_(origin),
        m_T_w_(w_T_m_.inverse()),
        data_(height * width, default_value),
        data_ptr_(data_.data())
    {
    }

    Gridmap(const double origin_x,
            const double origin_y,
            const double origin_phi,
            const double resolution,
            const std::size_t height,
            const std::size_t width,
            const T &default_value,
            const std::string &frame) :
        Map2D(frame),
        resolution_(resolution),
        resolution_inv_(1.0 / resolution),
        height_(height),
        width_(width),
        max_index_({(int)(width)-1,(int)(height)-1}),
        w_T_m_(origin_x, origin_y, origin_phi),
        m_T_w_(w_T_m_.inverse()),
        data_(height * width, default_value),
        data_ptr_(data_.data())
    {
    }

    Gridmap(const Gridmap &other) :
        Map2D(other.frame_),
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        height_(other.height_),
        width_(other.width_),
        max_index_(other.max_index_),
        w_T_m_(other.w_T_m_),
        m_T_w_(other.m_T_w_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }

    Gridmap(Gridmap &&other) :
        Map2D(other.frame_),
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        height_(other.height_),
        width_(other.width_),
        max_index_(other.max_index_),
        w_T_m_(other.w_T_m_),
        m_T_w_(other.m_T_w_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }


    virtual ~Gridmap()
    {
    }

    virtual inline cslibs_math_2d::Point2d getMin() const override
    {
        cslibs_math_2d::Point2d p;
        fromIndex({0,0},p);
        return p;
    }

    virtual inline cslibs_math_2d::Point2d getMax() const override
    {
        cslibs_math_2d::Point2d p;
        fromIndex({(int)width_-1,(int)height_-1},p);
        return p;
    }

    virtual inline cslibs_math_2d::Pose2d getOrigin() const
    {
        return w_T_m_;
    }

    inline T& at(const std::size_t idx, const std::size_t idy)
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline const T& at(const std::size_t idx, const std::size_t idy) const
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline T& at(const std::size_t i)
    {
        return data_ptr_[i];
    }

    inline const T& at(const std::size_t i) const
    {
        return data_ptr_[i];
    }

    virtual inline T& at(const cslibs_math_2d::Point2d &point)
    {
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    virtual inline T at(const cslibs_math_2d::Point2d &point) const
    {
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    inline const_line_iterator_t getConstLineIterator(const index_t &start,
                                                      const index_t &end) const
    {
        /// do index capping
        if(invalid(start)) {
            throw std::runtime_error("[GridMap]: Start index is invalid!");
        }
        if(invalid(end)) {
            throw std::runtime_error("[GridMap]: End index is invalid!");
        }
        return const_line_iterator_t(start, end, width_, data_ptr_);
    }

    inline const_line_iterator_t getConstLineIterator(const cslibs_math_2d::Point2d &start,
                                                      const cslibs_math_2d::Point2d &end) const
    {
        index_t start_index;
        index_t end_index;
        const bool start_in_bounds = toIndex(start, start_index);
        const bool end_in_bounds = toIndex(end, end_index);
//        if(start_in_bounds && !end_in_bounds) {
//            /// do index capping
//            if(start_index[0] == end_index[0]) {
//                /// cap y
//                end_index[1] = std::min(max_index_[1], std::max(0, end_index[1]));
//            } else if(start_index[1] == end_index[1]) {
//                /// cap x
//                end_index[1] = std::min(max_index_[0], std::max(0, end_index[0]));
//            } else {

//            }
//        }

        return const_line_iterator_t(start_index,
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

    std::vector<T> & getData()
    {
        return data_;
    }

    std::vector<T> const & getData() const
    {
        return data_;
    }


protected:
    const double                            resolution_;
    const double                            resolution_inv_;
    const std::size_t                       height_;
    const std::size_t                       width_;
    const index_t                           max_index_;
    const cslibs_math_2d::Transform2d       w_T_m_;
    const cslibs_math_2d::Transform2d       m_T_w_;

    std::vector<T>                          data_;
    T*                                      data_ptr_;

    inline bool invalid(const index_t &_i) const
    {
        return _i[0] < 0 ||
               _i[1] < 0 ||
               _i[0] > max_index_[0] ||
               _i[1] > max_index_[1];
    }

    inline bool toIndex(const cslibs_math_2d::Point2d &p_w,
                        index_t &i) const
    {
        const cslibs_math_2d::Point2d p_m = m_T_w_ * p_w;

        i[0] = static_cast<int>(p_m.x() * resolution_inv_ + 0.5);
        i[1] = static_cast<int>(p_m.y() * resolution_inv_ + 0.5);

        return (i[0] >= 0 && i[0] <= max_index_[0]) ||
               (i[1] >= 0 && i[1] <= max_index_[1]);
    }

    inline void fromIndex(const index_t &i,
                          cslibs_math_2d::Point2d &p_w) const
    {
        p_w = w_T_m_ * cslibs_math_2d::Point2d(i[0] * resolution_,
                                               i[1] * resolution_);
    }

    inline void fromIndex(const const_line_iterator_t &it,
                          cslibs_math_2d::Point2d &p_w) const
    {
        p_w = w_T_m_ * cslibs_math_2d::Point2d(it.x() * resolution_,
                                               it.y() * resolution_);
    }

    };

}
}
#endif /* STATIC_GRIDMAP_HPP */
