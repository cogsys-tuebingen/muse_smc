#ifndef SAMPLE_STATE_ITERATOR_HPP
#define SAMPLE_STATE_ITERATOR_HPP

/// CSLIBS
#include <cslibs_utility/buffered/buffered_vector.hpp>
#include <cslibs_time/time.hpp>

/// PROJECT
#include <muse_smc/smc/traits.hpp>

namespace muse_smc {
template<typename sample_t>
class StateIterator : public std::iterator<std::random_access_iterator_tag, typename traits::State<sample_t>::type>
{
public:
    using parent    = std::iterator<std::random_access_iterator_tag, typename traits::State<sample_t>::type>;
    using reference = typename parent::reference;

    inline explicit StateIterator(sample_t *begin) :
        data_(begin)
    {
    }

    virtual ~StateIterator() = default;

    inline StateIterator& operator++()
    {
        ++data_;
        return *this;
    }

    inline bool operator ==(const StateIterator<sample_t> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const StateIterator<sample_t> &_other) const
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
    sample_t        *data_;
};

template<typename sample_t>
class StateIteration
{
public:
    using sample_vector_t   = cslibs_utility::buffered::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using iterator_t        = StateIterator<sample_t>;
    using time_t            = cslibs_time::Time;

    inline StateIteration(const time_t &stamp,
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

    inline const time_t &getStamp() const
    {
        return stamp_;
    }

private:
    const time_t     stamp_;
    sample_vector_t &data_;
};
}

#endif // SAMPLE_STATE_ITERATOR_HPP
