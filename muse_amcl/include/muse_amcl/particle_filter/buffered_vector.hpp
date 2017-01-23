#ifndef BUFFERED_VECTOR_HPP
#define BUFFERED_VECTOR_HPP

#include <vector>
#include <memory>
#include <assert.h>

namespace std {
template<typename T>
class buffered_vector {
public:
    using Ptr            = std::shared_ptr<buffered_vector>;
    using iterator       = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;

    buffered_vector() :
        size_(0),
        data_ptr_(nullptr)
    {
    }
    /**
     * @brief buffered_vector constructor only with size where
     *        capacity = size.
     * @param size - the preferred size
     */
    buffered_vector(const std::size_t size) :
        size_(size),
        data_(size_, T()),
        data_ptr_(data_.data())
    {

    }

    /**
     * @brief buffered_vector constructor for preferred size and capacity.
     * @param size      - preferred value <= capacity
     * @param capacity  - the maximim capacity
     */
    buffered_vector(const std::size_t size,
                    const std::size_t capacity) :
        size_(size),
        data_(capacity, T()),
        data_ptr_(data_.data())
    {
        assert(size <= capacity);
    }

    /**
     * @brief Return reference to the i-th entry of the vector.
     *        This operator does not assure validity.
     * @param i - the index
     * @return  the i-th entry
     */
    T&  operator [] (const std::size_t i)
    {
        return data_ptr_[i];
    }

    /**
     * @brief Return constant reference to the i-th entry of the vector.
     *        This operator does not assure validity.
     * @param i - the index
     * @return  the i-th entry
     */
    const T&  operator [] (const std::size_t i) const
    {
        return data_ptr_[i];
    }

    /**
     * @brief Return reference to the i-th entry of the vector.
     * @param i - the index
     * @return  the i-th entry
     */
    T&  at (const std::size_t i)
    {
        if(i >= size_) {
            throw std::runtime_error("Index out of bounds!");
        }
        return data_ptr_[i];
    }

    /**
     * @brief Return constant reference to the i-th entry of the vector.
     * @param i - the index
     * @return  the i-th entry
     */
    const T&  at (const std::size_t i) const
    {
        if(i >= size_) {
            throw std::runtime_error("Index out of bounds!");
        }
        return data_ptr_[i];
    }

    /**
     * @brief Return the front of the buffered vector.
     * @return the back
     */
    inline T& front()
    {
        return data_.front();
    }

    /**
     * @brief Return the back of the buffered vector.
     * @return the back
     */
    inline T& back()
    {
        return data_.at(size_ - 1);
    }

    /**
     * @brief Return the iterator pointing towards the beginning of the vector.
     * @return  the begin iterator
     */
    inline iterator begin()
    {
        data_.begin();
    }

    /**
     * @brief Return the const_iterator pointing towards the beginning of the vector.
     * @return  the begin const_iterator
     */
    inline const_iterator begin() const
    {
        return data_.begin();
    }

    /**
     * @brief Iterator pointing towards the end. One element past the
     *        last one.
     * @return the end iterator.
     */
    inline iterator end()
    {
        return data_.begin() + size_;
    }

    /**
     * @brief Return the const_iterator pointing towards the end.
     * @return the end const_iterator
     */
    inline const_iterator end() const
    {
        return data_.begin() + size_;
    }

    /**
     * @brief Iterator pointing towards the end. One element past the
     *        last one.
     * @return the end iterator.
     */
    inline iterator limit()
    {
        return data_.end();
    }

    /**
     * @brief Return the const_iterator pointing towards the end.
     * @return the end const_iterator
     */
    inline const_iterator limit() const
    {
        return data_.end();
    }


    /**
     * @brief Return the maximum capacity of the buffered vector.
     * @return  the capacity
     */
    inline std::size_t capacity() const
    {
        return data_.size();
    }

    /**
     * @brief Return the current size of the vector contents.
     * @return  the size
     */
    inline std::size_t size() const
    {
        return size_;
    }


    /**
     * @brief Resize only given size, so that capacity is set to size.
     * @param size  - the preferred / maximal size of the vector
     */
    inline void resize(const std::size_t size)
    {
        size_ = size;
        data_.resize(size, T());
        data_ptr_ = data_.data();

    }

    /**
     * @brief Resize given size and capacity will generate buffered vector
     *        of desired size and capacity.
     * @param size      - the preferred size
     * @param capacity  - the preferred capacity.
     */
    inline void resize(const std::size_t size,
                       const std::size_t capacity)
    {
        size_ = size;
        data_.resize(capacity);
        data_ptr_ = data_.data();
    }

    /**
     * @brief Emplace value at the end within the given capacity.
     * @param value - the value to be emplaced.
     */
    inline void emplace_back(T& value)
    {
        if(size_ == data_.size()) {
            throw std::runtime_error("Buffered vector reached the capacity limit!");
        }
        data_ptr_[size_] = std::move(value);
        ++size_;
    }

    inline void push_back(const T &value)
    {
        if(size_ == data_.size()) {
            throw std::runtime_error("Buffered vector reached the capacity limit!");
        }
        data_ptr_[size_] = value;
        ++size_;
    }

    /**
     * @brief Clear the vector.
     */
    inline void clear()
    {
        size_ = 0;
        // let's spare some time
    //    std::fill(data_.begin(), data_.end(), T());
    }

private:
    std::size_t    size_;
    std::vector<T> data_;
    T*             data_ptr_;


};
}

#endif // BUFFERED_VECTOR_HPP
