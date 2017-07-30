#ifndef DISTANCE_TRANSFORM_HPP
#define DISTANCE_TRANSFORM_HPP
#include <muse_mcl/map/map.hpp>

#include <array>
#include <queue>

namespace muse_mcl {
namespace algorithms {
namespace distance_transform {
struct DistanceCache {
    std::vector<double> distances;
    const std::size_t   size;

    DistanceCache(const double resolution,
                  const double max_distance) :
        size(max_distance / resolution + 2 /*padding*/)
    {
        distances.resize(size * size, 0.0);
        for(std::size_t i = 0 ; i < size ; ++i) {
            for(std::size_t j = 0 ; j < size ; ++j) {
                distances[i * size + j] = std::hypot(i,j) * resolution;
            }
        }
    }

    double operator () (const std::size_t x,
                        const std::size_t y)
    {
        return distances[y * size + x];
    }

};
struct Data {
    struct Less {
        bool operator()( const Data& lhs,
                         const Data& rhs ) const
        {
            return *lhs.data <
                    *rhs.data;
        }
    };

    inline Data(const std::size_t     x,
                const std::size_t     y,
                const double         *data) :
        x(x),y(y),
        src_x(x),src_y(y),
        data(data)
    {
    }


    inline Data(const std::size_t x,
                const std::size_t y,
                const std::size_t src_x,
                const std::size_t src_y,
                const double *data) :
        x(x),y(y),
        src_x(src_x),src_y(src_y),
        data(data)
    {
    }

    inline Data(const Data &other) :
        x(other.x),y(other.y),
        src_x(other.src_x),src_y(other.src_y),
        data(other.data)
    {
    }

    inline Data(Data &&other) :
        x(other.x),y(other.y),
        src_x(other.src_x),src_y(other.src_y),
        data(other.data)
    {
    }

    inline Data & operator = (const Data &other)
    {
        if(&other != this) {
            x = other.x;
            y = other.y;
            src_x = other.src_x;
            src_y = other.src_y;
            data = other.data;
        }
        return *this;
    }

    std::size_t dx() const
    {
        return std::abs(static_cast<int>(x) - static_cast<int>(src_x));
    }

    std::size_t dy() const
    {
        return std::abs(static_cast<int>(y) - static_cast<int>(src_y));
    }


    std::size_t          x, y;
    std::size_t          src_x, src_y;
    const double        *data;

};
}


template<typename T>
class DistanceTransform {
public:
    inline DistanceTransform(const double resolution,
                             const double maximum_distance,
                             const T occupancy_threshold) :
        resolution_(resolution),
        maximum_distance_(maximum_distance),
        occupancy_threshold_(occupancy_threshold),
        cache_(resolution, maximum_distance)
    {
    }

    inline void apply(const std::vector<T> &src,
                      const std::size_t step,
                      std::vector<double> &dst)
    {
        src_size_   = src.size();
        src_width_  = step;
        src_height_ = src.size() / step;

        queue_ = std::priority_queue<cell_data_t, std::deque<cell_data_t>, cell_data_t::Less>();
        marked_.resize(src.size(), 0);
        dst.resize(src.size(), maximum_distance_);

        /// prepare the distance map
        for(std::size_t y = 0ul ; y < src_height_ ; ++y) {
            for(std::size_t x = 0ul ; x < src_width_ ; ++x) {
                const std::size_t pos = y * src_width_ + x;
                if(src[pos] >= occupancy_threshold_) {
                    queue_.emplace(cell_data_t(x,y,dst.data() + pos));
                    dst[pos] = 0.0;
                    marked_[pos] = 1;
                }
            }
        }

        while(!queue_.empty()) {
            cell_data_t cell = queue_.top();
            if(cell.x > 0) {
                auto c = cell_data_t(cell.x - 1ul, cell.y, cell.src_x, cell.src_y,
                                     dst.data() + cell.x - 1ul + cell.y * src_width_);
                enqueue(dst, c);
            }
            if(cell.y > 0) {
                auto c =cell_data_t(cell.x, cell.y - 1ul, cell.src_x, cell.src_y,
                                    dst.data() + cell.x + (cell.y - 1ul) * src_width_);
                enqueue(dst, c);
            }
            if(cell.x < src_width_ - 1ul) {
                auto c = cell_data_t(cell.x + 1ul, cell.y, cell.src_x, cell.src_y,
                                     dst.data() + cell.x + 1ul + cell.y * src_width_);
                enqueue(dst, c);
            }
            if(cell.y < src_height_ - 1ul) {
                auto c = cell_data_t(cell.x, cell.y + 1ul, cell.src_x, cell.src_y,
                                     dst.data() + cell.x + (cell.y + 1ul) * src_width_);
                enqueue(dst, c);
            }
            queue_.pop();
        }
    }

private:
    using cell_data_t = distance_transform::Data;

    const double            resolution_;
    const double            maximum_distance_;
    const T                 occupancy_threshold_;

    std::size_t                         src_size_;
    std::size_t                         src_width_;
    std::size_t                         src_height_;

    distance_transform::DistanceCache   cache_;
    std::vector<uint8_t>                marked_;

    std::priority_queue<cell_data_t, std::deque<cell_data_t>, cell_data_t::Less> queue_;

    inline void enqueue(std::vector<double> &dst,
                        cell_data_t         &cell)
    {
        if(marked_[cell.y * src_width_ + cell.x])
            return;
        const std::size_t dx = cell.dx();
        const std::size_t dy = cell.dy();
        const double d = cache_(dx, dy);

        if(d > maximum_distance_)
            return;

        dst[cell.x + cell.y * src_width_] = d;
        marked_[cell.x + cell.y * src_width_] = 1;
        queue_.emplace(cell);
    }

};
}
}

#endif // DISTANCE_TRANSFORM_HPP
