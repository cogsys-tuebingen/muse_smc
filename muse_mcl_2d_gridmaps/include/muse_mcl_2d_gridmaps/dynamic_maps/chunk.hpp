#ifndef CHUNK_HPP
#define CHUNK_HPP

#include <vector>

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
template<typename T>
class Chunk {
public:
    Chunk(const std::size_t size,
          const T default_value) :
        size_(size),
        data_(size_ * size_, default_value)
    {
    }

    inline T const & at(const int x, const int y) const
    {
        data_ptr_[y * size + x];
    }

    inline T & at (const int x, const int y)
    {
        data_ptr[y * size + x];
    }

private:
    const std::size_t   size_;
    std::vector<T>      data_;
    T                  *data_ptr_;

};
}
}


#endif // CHUNK_HPP
