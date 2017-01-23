#ifndef PARTICLE_SET_MEMBER_ITERATOR_HPP
#define PARTICLE_SET_MEMBER_ITERATOR_HPP

#include "buffered_vector.hpp"

namespace muse_amcl {
template<typename Data, typename T, T Data::*Member, typename Notifier>
class MemberIterator : public std::iterator<std::random_access_iterator_tag, T>
{
    using parent    = std::iterator<std::random_access_iterator_tag, T>;
    using iterator  = typename parent::iterator;
    using reference = typename parent::reference;

    using notify_update = void (Notifier::*)(const T &);

    Data            *data_;
    Notifier        &notifier_;
    notify_update    update_;

public:
    explicit MemberIterator(Data         *begin,
                            Notifier     &notifier,
                            notify_update update) :
        data_(begin),
        notifier_(notifier),
        update_(update)
    {
    }

    inline iterator& operator++()
    {
        (notifier_.*update_)(data_->*Member);
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
};

template<typename Data, typename T, T Data::*Member, typename Notifier>
class MemberDecorator {
public:
    using iterator = MemberIterator<Data, T, Member, Notifier>;
    using Notification   = void (Notifier::*)(const T &);

    MemberDecorator(std::buffered_vector<Data> &data,
                    Notifier                   &notifier,
                    Notification                notification) :
        data_(data),
        notifier_(notifier),
        notification_(notification)
    {
    }

    inline iterator begin()
    {
        return iterator(&data_.front(), notifier_, notification_);
    }

    inline iterator end() {
        return iterator(&data_.back() + 1, notifier_, notification_);   /// end has always to be set off by 1
    }
private:
    std::buffered_vector<Data> &data_;
    Notifier                   &notifier_;
    Notification                notification_;
};


}



#endif // PARTICLE_SET_MEMBER_ITERATOR_HPP
