#ifndef SAMPLE_WEIGHT_ITERATOR_HPP
#define SAMPLE_WEIGHT_ITERATOR_HPP

#include <muse_smc/utility/buffered_vector.hpp>
#include <muse_smc/utility/delegate.hpp>

namespace muse_smc {
template<typename sample_t>
class WeightIterator : public std::iterator<std::random_access_iterator_tag, double>
{
public:
    using parent    = std::iterator<std::random_access_iterator_tag, double>;
    using iterator  = typename parent::iterator;
    using reference = typename parent::reference;

    using notify_update = delegate<void(const double &)>;


    inline explicit WeightIterator(sample_t        *begin,
                                   notify_update    update) :
        data_(begin),
        update_(update)
    {
    }

    inline iterator& operator++()
    {
        update_(data_->weight);
        ++data_;
        return *this;
    }

    inline bool operator ==(const WeightIterator<sample_t> &_other) const
    {
        return data_ == _other.data_;
    }

    inline bool operator !=(const WeightIterator<sample_t> &_other) const
    {
        return !(*this == _other);
    }


    inline reference operator *() const
    {
        return data_->weight;
    }

    inline const sample_t& getData() const
    {
        return *data_;
    }

private:
    sample_t        *data_;
    notify_update    update_;
};

template<typename sample_t>
class WeightIteration
{
public:
    using sample_vector_t = std::buffered_vector<sample_t, typename sample_t::allocator_t>;
    using notify_update   = delegate<void(const double)>;
    using notify_touch    = delegate<void()>;
    using notify_finished = delegate<void()>;
    using iterator_t      = WeightIterator<sample_t>;

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

    inline virtual ~WeightIteration()
    {
        if(!untouched_)
            finish_();
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

    inline const sample_vector_t& getData() const
    {
        return data_;
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
