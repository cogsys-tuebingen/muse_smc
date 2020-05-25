#ifndef MUSE_SMC_SAMPLE_STATE_ITERATOR_HPP
#define MUSE_SMC_SAMPLE_STATE_ITERATOR_HPP

/// CSLIBS
#include <cslibs_utility/buffered/buffered_vector.hpp>

/// PROJECT
#include <muse_smc/smc/traits/sample.hpp>

namespace muse_smc {
template<typename Sample_T>
class StateIterator : public std::iterator<std::random_access_iterator_tag, typename traits::State<Sample_T>::type>
{
public:
    using parent    = std::iterator<std::random_access_iterator_tag, typename traits::State<Sample_T>::type>;
    using reference = typename parent::reference;

    inline explicit StateIterator(Sample_T *begin) :
        data_(begin)
    {
    }

    virtual ~StateIterator() = default;

    inline StateIterator& operator++()
    {
        ++data_;
        return *this;
    }

    inline bool operator ==(const StateIterator<Sample_T> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const StateIterator<Sample_T> &_other) const
    {
        return !(*this == _other);
    }

    inline reference operator *() const
    {
        return data_->state;
    }

    inline double weight() const
    {
        return data_->weight;
    }

private:
    Sample_T        *data_;
};

template<typename Sample_T, typename Time_T>
class StateIteration
{
public:
    using sample_vector_t   = cslibs_utility::buffered::buffered_vector<Sample_T, typename Sample_T::allocator_t>;
    using iterator_t        = StateIterator<Sample_T>;

    inline StateIteration(const Time_T &stamp,
                          sample_vector_t &data) :
        stamp_(stamp),
        data_(data)
    {
    }

    virtual ~StateIteration()
    {
    }

    inline iterator_t begin()
    {
        return iterator_t(&(*data_.begin()));
    }

    inline iterator_t end() {
        return iterator_t(&(*data_.end()));
    }

    inline const sample_vector_t& getData() const
    {
        return data_;
    }

    inline const Time_T &getStamp() const
    {
        return stamp_;
    }

private:
    const Time_T     stamp_;
    sample_vector_t &data_;
};
}

#endif // MUSE_SMC_SAMPLE_STATE_ITERATOR_HPP
