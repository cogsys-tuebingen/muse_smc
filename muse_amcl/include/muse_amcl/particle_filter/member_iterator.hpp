#ifndef PARTICLE_SET_MEMBER_ITERATOR_HPP
#define PARTICLE_SET_MEMBER_ITERATOR_HPP

#include <muse_amcl/utils/buffered_vector.hpp>
#include <muse_amcl/utils/delegate.hpp>

namespace muse_amcl {
template<typename Data, typename T, T Data::*Member, typename Notifier>
class MemberIterator : public std::iterator<std::random_access_iterator_tag, T>
{
    using parent    = std::iterator<std::random_access_iterator_tag, T>;
    using iterator  = typename parent::iterator;
    using reference = typename parent::reference;

    using notify_update = delegate<void(const T &)>;

    Data            *data_;
    notify_update    update_;

public:
    explicit MemberIterator(Data         *begin,
                            notify_update update) :
        data_(begin),
        update_(update)
    {
    }

    inline iterator& operator++()
    {
        update_(data_->*Member);
        ++data_;
        return *this;
    }

    inline bool operator ==(const MemberIterator<Data, T, Member, Notifier> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const MemberIterator<Data, T, Member, Notifier> &_other) const
    {
        return !(*this == _other);
    }

    inline reference operator *() const
    {
        return (data_->*Member);
    }

    inline const Data& getData() const
    {
        return *data_;
    }

};

template<typename Data, typename T, T Data::*Member, typename Notifier>
class MemberDecorator {
public:
    using iterator = MemberIterator<Data, T, Member, Notifier>;
    using notify_update = delegate<void(const T&)>;

    MemberDecorator(std::buffered_vector<Data> &data,
                    notify_update                update) :
        data_(data),
        update_(update)
    {
    }

    inline iterator begin()
    {
        return iterator(&data_.front(), update_);
    }

    inline iterator end() {
        return iterator(&data_.back() + 1, update_);   /// end has always to be set off by 1
    }
private:
    std::buffered_vector<Data> &data_;
    notify_update               update_;
};


}



#endif // PARTICLE_SET_MEMBER_ITERATOR_HPP
