#ifndef SAMPLE_INSERTION_HPP
#define SAMPLE_INSERTION_HPP

#include <cslibs_utility/buffered/buffered_vector.hpp>
#include <cslibs_utility/common/delegate.hpp>

namespace muse_smc {
/**
 * @brief The SampleInsertion class is used to fill up a particle set.
 *        The insertion object notifies usage and changes.
 */
template<typename sample_t>
class SampleInsertion {
public:
    using notify_closed   = cslibs_utility::common::delegate<void()>;
    using notify_update   = cslibs_utility::common::delegate<void(const sample_t &)>;
    using sample_vector_t = cslibs_utility::buffered::buffered_vector<sample_t, typename sample_t::allocator_t>;

    /**
     * @brief Insertion constructor.
     * @param data      - data structure to insert to
     * @param update    - on change notification callback
     * @param finshed   - on finish callback
     */
    inline SampleInsertion(sample_vector_t &data,
                           notify_update    update,
                           notify_closed    close) :
        data_(data),
        open_(true),
        update_(update),
        close_(close)
    {
    }

    virtual ~SampleInsertion()
    {
        if(open_) {
            if(touched_)
                close_();
        }
    }

    inline void insert(sample_t&& sample)
    {
        if(!open_)
            return;

        touched_ = true;

        data_.emplace_back(std::move(sample));
        update_(data_.back());
    }

    inline void insert(const sample_t &sample)
    {
        if(!open_)
            return;

        touched_ = true;

        data_.push_back(sample);
        update_(data_.back());
    }

    inline bool canInsert() const
    {
        return data_.size() < data_.capacity() && open_;
    }

    inline void close()
    {
        if(open_) {
            open_ = false;

            if(touched_)
                close_();
        }
    }

    sample_vector_t const & getData()
    {
        return data_;
    }

private:
    sample_vector_t &data_;       /// the data container

    bool             open_;       /// indicator if insertion is still open
    bool             touched_;    /// indicator if something was inserted
    notify_update    update_;     /// on update callback
    notify_closed    close_;      /// on close / finish callback
};
}

#endif // SAMPLE_INSERTION_HPP
