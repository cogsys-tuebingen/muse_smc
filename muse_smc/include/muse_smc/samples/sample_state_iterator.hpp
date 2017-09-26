#ifndef SAMPLE_STATE_ITERATOR_HPP
#define SAMPLE_STATE_ITERATOR_HPP

#include <muse_smc/utility/buffered_vector.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class StateIterator : public std::iterator<std::random_access_iterator_tag, typename state_space_description_t::state_t>
{
public:
    using parent    = std::iterator<std::random_access_iterator_tag, typename state_space_description_t::state_t>;
    using sample_t  = typename state_space_description_t::sample_t;
    using iterator  = typename parent::iterator;
    using reference = typename parent::reference;

    inline explicit StateIterator(sample_t *begin) :
        data_(begin)
    {
    }

    inline iterator& operator++()
    {
        ++data_;
        return *this;
    }

    inline bool operator ==(const StateIterator<state_space_description_t> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const StateIterator<state_space_description_t> &_other) const
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

template<typename state_space_description_t>
class StateIteration
{
public:
    using sample_t          = typename state_space_description_t::sample_t;
    using sample_vector_t   = std::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using iterator_t        = StateIterator<state_space_description_t>;

    inline StateIteration(sample_vector_t &data) :
        data_(data)
    {
    }

    inline virtual ~StateIteration()
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

private:
    sample_vector_t &data_;
};
}


#endif // SAMPLE_STATE_ITERATOR_HPP
