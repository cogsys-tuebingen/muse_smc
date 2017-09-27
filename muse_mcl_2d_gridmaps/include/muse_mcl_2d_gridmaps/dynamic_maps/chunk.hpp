#ifndef CHUNK_HPP
#define CHUNK_HPP

#include <vector>
#include <array>
#include <mutex>

namespace muse_mcl_2d_gridmaps {
namespace dynamic_maps {
template<typename T>
class Chunk {
public:
    using index_t = std::array<int, 2>;
    using mutex_t = std::mutex;

    Chunk() = default;
    virtual ~Chunk() = default;

    Chunk(const int size,
          const T default_value) :
        size_(size),
        data_(size * size, default_value),
        data_ptr_(data_.data())
    {
    }

    Chunk(const Chunk &other) :
        size_(other.size_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }

    Chunk(Chunk &&other) :
        size_(other.size_),
        data_(std::move(other.data_)),
        data_ptr_(data_.data())
    {
    }

    Chunk& operator = (const Chunk &other)
    {
        size_  = (other.size_);
        data_  = (other.data_);
        data_ptr_ = (data_.data());
        return *this;
    }

    inline T const & at(const index_t &i) const
    {
        return data_ptr_[i[1] * size_ + i[0]];
    }

    inline T & at (const index_t &i)
    {
        return data_ptr_[i[1] * size_ + i[0]];
    }

    inline T & at (const int idx, const int idy)
    {
        return data_ptr_[idy * size_ + idx];
    }

    inline T const & at (const int idx, const int idy) const
    {
        return data_ptr_[idy * size_ + idx];
    }

    inline void merge(const Chunk &other)
    {
    }

private:
    int                 size_;
    std::vector<T>      data_;
    T                  *data_ptr_;

};
}
}


#endif // CHUNK_HPP
