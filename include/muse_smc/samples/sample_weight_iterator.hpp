#ifndef SAMPLE_WEIGHT_ITERATOR_HPP
#define SAMPLE_WEIGHT_ITERATOR_HPP

#include <cslibs_utility/buffered/buffered_vector.hpp>
#include <cslibs_utility/common/delegate.hpp>

namespace muse_smc {
template<typename state_space_description_t>
class WeightIterator : public std::iterator<std::random_access_iterator_tag, double>
{
public:
    using state_t       = typename state_space_description_t::state_t;
    using sample_t      = typename state_space_description_t::sample_t;
    using parent        = std::iterator<std::random_access_iterator_tag, double>;
    using iterator      = typename parent::iterator;
    using reference     = typename parent::reference;
    using notify_update = cslibs_utility::common::delegate<void(const double &)>;

    inline explicit WeightIterator(sample_t      *begin,
                                   notify_update  update) :
        data_(begin),
        update_(update)
    {
    }

    virtual ~WeightIterator() = default;

    inline iterator& operator++()
    {
        update_(data_->weight);
        ++data_;
        return *this;
    }

    inline bool operator ==(const WeightIterator &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const WeightIterator &_other) const
    {
        return !(*this == _other);
    }


    inline reference operator *() const
    {
        return data_->weight;
    }

    inline const state_t& state() const
    {
        return data_->state;
    }

private:
    sample_t        *data_;
    notify_update    update_;
};

template<typename state_space_description_t>
class WeightIteration
{
public:
    using sample_t          = typename state_space_description_t::sample_t;
    using sample_vector_t   = cslibs_utility::buffered::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using notify_update     = cslibs_utility::common::delegate<void(const double)>;
    using notify_touch      = cslibs_utility::common::delegate<void()>;
    using notify_finished   = cslibs_utility::common::delegate<void()>;
    using iterator_t        = WeightIterator<state_space_description_t>;
    using const_iterator_t  = typename sample_vector_t::const_iterator;

    inline WeightIteration(sample_vector_t &data,
                           notify_touch     touch,
                           notify_update    update,
                           notify_finished  finish) :
        data_(data),
        touch_(touch),
        update_(update),
        finish_(finish),
        untouched_(true)
    {
    }

    virtual ~WeightIteration()
    {
        if(!untouched_)
            finish_();
    }

    inline const_iterator_t const_begin() const
    {
        return data_.begin();
    }

    inline const_iterator_t const_end() const
    {
        return data_.begin();
    }

    inline iterator_t begin()
    {
        if(untouched_) {
            untouched_ = false;
            touch_();
        }

        return iterator_t(&(*data_.begin()), update_);
    }

    inline iterator_t end() {
        return iterator_t(&(*data_.end()), update_);
    }

    inline std::size_t size() const
    {
        return data_.size();
    }

    inline std::size_t capacity() const
    {
        return data_.capacity();
    }

private:
    sample_vector_t &data_;
    notify_touch     touch_;
    notify_update    update_;
    notify_finished  finish_;
    bool             untouched_;
};
}

#endif // SAMPLE_WEIGHT_ITERATOR_HPP
