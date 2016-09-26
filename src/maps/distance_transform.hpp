#pragma once

#include <cmath>
#include <vector>
#include <assert.h>
#include <limits>

namespace muse {
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

template<typename T>
struct Borgefors {
    Borgefors(const std::size_t _rows,
              const std::size_t _cols,
              const double _resolution,
              const T _threshold,
              const std::size_t ksize) :
        kernel(ksize, _resolution),
        rows(_rows),
        cols(_cols),
        size(_rows * _cols),
        threshold(_threshold),
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
        threshold(_threshold)
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
                _dst[i] = std::numeric_limits<double>::max();
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
                        if(n < std::numeric_limits<double>::max()) {
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
                    if(n < std::numeric_limits<double>::max()) {
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
                    if(n < std::numeric_limits<double>::max()) {
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
                        if(n < std::numeric_limits<double>::max()) {
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
    const std::size_t max_idx;
    const std::size_t max_idy;

};

struct Dijkstra {



};
}
}
}
