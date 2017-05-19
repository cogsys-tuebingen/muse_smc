#ifndef DISTANCE_TRANSFORM_HPP
#define DISTANCE_TRANSFORM_HPP

#include <cmath>
#include <vector>
#include <assert.h>
#include <limits>
#include <queue>
#include <array>

namespace muse_mcl {
namespace maps {
namespace distance_transform {
class Kernel {
public:
    Kernel(const std::size_t size,
           const double scale = 1.0) :
        size(size),
        margin(size / 2),
        min(- (int) size / 2),
        max(size / 2),
        data(size *  size),
        data_ptr(data.data())
    {
        assert(size % 2 != 0);
        for(int i = min ; i <= max ; ++i) {
            for(int j = min ; j <= max ; ++j) {
                at(i,j) = scale * hypot(i,j);
            }
        }
    }

    inline double at(const int i, const int j) const
    {
        return data_ptr[(i - min) * size + (j - min)];
    }

    inline double at(const std::size_t i, const std::size_t j) const
    {
        return data_ptr[i * size + j];
    }

    inline bool inRange(const int id) const
    {
        return id >= min && id <= max;
    }

    const std::size_t size;
    const std::size_t margin;
    const int         min;
    const int         max;

private:
    std::vector<double> data;
    double             *data_ptr;

    inline double & at(const int i, const int j)
    {
        return data_ptr[(i - min) * size + (j - min)];
    }
};

/**
 * @brief The Borgefors struct implements the Borgefors distance transform algorithm.
 *        It can be used with different kernel sizes, wheareas it is recommended to use at least
 *        a kernel size of 5.
 *        This algorithm is not error free.
 */
template<typename T>
struct Borgefors {
    Borgefors(const std::size_t _rows,
              const std::size_t _cols,
              const double _resolution,
              const std::size_t ksize,
              const T _threshold) :
        kernel(ksize, _resolution),
        rows(_rows),
        cols(_cols),
        size(_rows * _cols),
        threshold(_threshold),
        max_distance(_resolution * hypot(rows, cols)),
        max_idx(_cols - 1),
        max_idy(_rows - 1)
    {
    }

    Borgefors(const std::size_t _rows,
              const std::size_t _cols,
              const double _resolution,
              const double radius,
              const T _threshold) :
        kernel(radius / _resolution, _resolution),
        rows(_rows),
        cols(_cols),
        size(_rows * _cols),
        threshold(_threshold),
        max_distance(_resolution * hypot(rows, cols)),
        max_idx(_cols - 1),
        max_idy(_rows - 1)
    {
    }


    void apply(const std::vector<T> &_src,
               std::vector<double> &_dst)
    {
        assert(_src.size() == _dst.size());
        assert(_src.size() == size);
        apply(_src.data(), _dst.data());
    }

    void apply(const T *_src, double *_dst)
    {
        assert(_src != nullptr);
        assert(_dst != nullptr);

        /// initialisation
        for(std::size_t i = 0 ; i < size ; ++i) {
            if(_src[i] < threshold) {
                _dst[i] = max_distance;
            } else {
                _dst[i] = 0;
            }
        }

        /// forward sweep
        for(std::size_t i = kernel.margin ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                double &dst = _dst[i * cols + j];

                for(int k = kernel.min ; k < 0 ; ++k) {
                    for(int l = kernel.min ; l <= kernel.max ; ++l) {
                        if(j + l < 0 || j + l > max_idx) {
                            continue;
                        }

                        double n = _dst[(i + k) * cols + (j + l)];
                        if(n < max_distance) {
                            n += kernel.at(k,l);
                        }

                        if(n < dst) {
                            dst = n;
                        }

                    }
                }

                for(int l = kernel.min ; l < 0 ; ++l) {
                    if(j + l < 0 || j + l > max_idx) {
                        continue;
                    }

                    double n = _dst[i * cols + (j + l)];
                    if(n < max_distance) {
                        n += kernel.at(0,l);
                    }

                    if(n < dst) {
                        dst = n;
                    }
                }
            }
        }

        /// backwards sweep
        for(int i = rows - (kernel.margin + 1) ; i >= 0 ; --i) {
            for(int j = max_idy ; j >= 0 ; --j) {
                double &dst = _dst[i * cols + j];

                for(int l = 1 ; l <= kernel.max ; ++l) {
                    if(j + l < 0 || j + l > max_idx) {
                        continue;
                    }

                    double n = _dst[i * cols + (j + l)];
                    if(n < max_distance) {
                        n += kernel.at(0,l);
                    }

                    if(n < dst) {
                        dst = n;
                    }
                }

                for(int k = 1 ; k <= kernel.max ; ++k) {
                    for(int l = kernel.min ; l <= kernel.max ; ++l) {
                        if(j + l < 0 || j + l > max_idx) {
                            continue;
                        }

                        double n = _dst[(i + k) * cols + (j + l)];
                        if(n < max_distance) {
                            n += kernel.at(k,l);
                        }

                        if(n < dst) {
                            dst = n;
                        }
                    }
                }
            }
        }
    }

    const Kernel      kernel;
    const std::size_t rows;
    const std::size_t cols;
    const std::size_t size;
    const T threshold;
    const double max_distance;
    const std::size_t max_idx;
    const std::size_t max_idy;

};

/**
 * @brief The ModifiedDijkstra struct implements the MD algorithm for distance transform.
 *        The algorithm performs similarly to the first one. Due to the graph-based
 *        implementation, there is not need for a kernel size anymore.
 */
template<typename T>
struct ModifiedDijkstra {
    struct Vertex {
        Vertex(const int i,
               const int j) :
            i(i),
            j(j)
        {
        }

        int i;
        int j;
    };


    ModifiedDijkstra(const std::size_t _rows,
             const std::size_t _cols,
             const double _resolution,
             const T _threshold) :
        kernel(3, _resolution),
        rows(_rows),
        cols(_cols),
        size(_rows * _cols),
        threshold(_threshold),
        max_distance(_resolution * hypot(rows, cols)),
        resolution(resolution),
        max_idx(_cols - 1),
        max_idy(_rows - 1)
    {
    }

    void apply(const std::vector<T> &_src,
               std::vector<double> &_dst)
    {
        assert(_src.size() == _dst.size());
        assert(_src.size() == size);
        apply(_src.data(), _dst.data());
    }

    void apply(const T *_src, double *_dst)
    {
        assert(_src != nullptr);
        assert(_dst != nullptr);

        /// the priority queue sorted from smallest to biggest
        auto less = [this, _dst]( const Vertex &a, const Vertex &b )
        { return _dst[a.i * cols + a.j] < _dst[b.i * cols + b.j]; };

        std::priority_queue< Vertex , std::vector<Vertex>, decltype( less ) > Q( less );

        const static std::array<int, 8> dy = {-1,-1,-1, 0, 0, 1, 1, 1};
        const static std::array<int, 8> dx = {-1, 0, 1,-1, 1,-1, 0, 1};
        /// initialisation
        double  *dst_ptr = _dst;
        const T *src_ptr = _src;
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                if(*src_ptr < threshold) {
                    *dst_ptr = max_distance;
                } else {
                    *dst_ptr = 0.0;
                     Q.emplace(Vertex(i,j));
                }
                ++src_ptr;
                ++dst_ptr;
            }
        }


        while(!Q.empty()) {
            Vertex v = Q.top();
            Q.pop();
            for(std::size_t n = 0 ; n < 8 ; ++n) {
                int i = v.i + dy[n];
                int j = v.j + dx[n];
                if(i >= 0 && i <= max_idy &&
                        j >= 0 && j <= max_idx) {
                    double d    = _dst[v.i * cols + v.j] + kernel.at(dy[n], dx[n]);
                    double &dst = _dst[i * cols + j];
                    if(d < dst) {
                        dst = d;
                        Q.emplace(Vertex(i,j));
                    }
                }
            }
        }
    }

    const Kernel      kernel;
    const std::size_t rows;
    const std::size_t cols;
    const std::size_t size;
    const T threshold;
    const double max_distance;
    const double resolution;
    const std::size_t max_idx;
    const std::size_t max_idy;

};

/**
 * @brief The ModifiedDijkstraDeadReckoning struct implements the MDDR which should be error free.
 *        Among the three it is the most accurate but also the slowest.
 */
template<typename T>
struct ModifiedDijkstraDeadReckoning {
    struct Vertex {
        Vertex(const int i,
               const int j) :
            i(i),
            j(j)
        {
        }

        int i;
        int j;
    };


    ModifiedDijkstraDeadReckoning(const std::size_t _rows,
                                  const std::size_t _cols,
                                  const double _resolution,
                                  const T _threshold) :
        kernel(3, _resolution),
        rows(_rows),
        cols(_cols),
        size(_rows * _cols),
        threshold(_threshold),
        max_distance(_resolution * hypot(rows, cols)),
        resolution(resolution),
        max_idx(_cols - 1),
        max_idy(_rows - 1)
    {
    }

    void apply(const std::vector<T> &_src,
               std::vector<double> &_dst)
    {
        assert(_src.size() == _dst.size());
        assert(_src.size() == size);
        apply(_src.data(), _dst.data());
    }

    void apply(const T *_src, double *_dst)
    {
        assert(_src != nullptr);
        assert(_dst != nullptr);

        /// the priority queue sorted from smallest to biggest
        auto less = [this, _dst]( const Vertex &a, const Vertex &b )
        { return _dst[a.i * cols + a.j] < _dst[b.i * cols + b.j]; };

        std::priority_queue< Vertex , std::vector<Vertex>, decltype( less ) > Q( less );
        std::vector<Vertex> B;

        const static std::array<int, 8> dy = {-1,-1,-1, 0, 0, 1, 1, 1};
        const static std::array<int, 8> dx = {-1, 0, 1,-1, 1,-1, 0, 1};
        /// initialisation
        double  *dst_ptr = _dst;
        const T *src_ptr = _src;
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                if(*src_ptr < threshold) {
                    *dst_ptr = max_distance;
                } else {
                    *dst_ptr = 0.0;
                    /// only if border point
                    bool is_border = false;
                    for(std::size_t n = 0 ; n < 8 ; ++n) {
                        int ni = i + dy[n];
                        int nj = j + dx[n];
                        if(ni >= 0 && ni <= max_idy &&
                                nj >= 0 && nj <= max_idx) {
                            is_border |= _src[ni * cols + nj] < threshold;
                            if(is_border)
                                break;
                        }
                    }
                    if(is_border) {
                        Q.emplace(Vertex(i,j));
                        B.emplace_back(Vertex(i,j));
                    }
                }
                ++src_ptr;
                ++dst_ptr;
            }
        }


        while(!Q.empty()) {
            Vertex v = Q.top();
            Q.pop();
            for(std::size_t n = 0 ; n < 8 ; ++n) {
                int i = v.i + dy[n];
                int j = v.j + dx[n];
                if(i >= 0 && i <= max_idy &&
                        j >= 0 && j <= max_idx) {
                    double d    = _dst[v.i * cols + v.j] + kernel.at(dy[n], dx[n]);
                    double &dst = _dst[i * cols + j];
                    if(d < dst) {
                        dst = d;
                        Q.emplace(Vertex(i,j));
                    } else {
                        d = std::numeric_limits<double>::max();
                        for(Vertex &b : B) {
                            double db = hypot(b.i - i, b.j - j);
                            if(db < d)
                                d = db;
                        }
                        dst = d;
                    }
                }
            }
        }
    }

    const Kernel      kernel;
    const std::size_t rows;
    const std::size_t cols;
    const std::size_t size;
    const T threshold;
    const double max_distance;
    const double resolution;
    const std::size_t max_idx;
    const std::size_t max_idy;

};
}
}
}

#endif /* DISTANCE_TRANSFORM_HPP */
