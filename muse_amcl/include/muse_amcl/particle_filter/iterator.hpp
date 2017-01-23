#ifndef ITERATOR_HPP
#define ITERATOR_HPP

#include "buffered_vector.hpp"

namespace muse_amcl {
template<typename Data, typename Notifier>
class Iterator : public std::iterator<std::random_access_iterator_tag, Data>
{
    using parent    = std::iterator<std::random_access_iterator_tag, Data>;
    using iterator  = typename parent::iterator;
    using reference = typename parent::reference;

    Data     *data_;
    Notifier &notifier_;


public:
    explicit Iterator(Data      *begin,
                      Notifier  &notifier) :
        data_(begin),
        notifier_(notifier)
    {
    }

    inline iterator& operator++()
    {
        notifier_.notify(*data_);
        ++data_;
        return *this;
    }

    inline bool operator ==(const Iterator<Data, Notifier> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const Iterator<Data, Notifier> &_other) const
    {
        return !(*this == _other);
    }

    inline reference operator *() const
    {
        return *data_;
    }
};
}

#endif // ITERATOR_HPP
