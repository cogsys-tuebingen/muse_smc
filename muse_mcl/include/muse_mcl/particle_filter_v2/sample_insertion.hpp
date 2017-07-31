#ifndef SAMPLE_INSERTION_HPP
#define SAMPLE_INSERTION_HPP

#include <muse_mcl/particle_filter_v2/sample.hpp>
#include <muse_mcl/utility/buffered_vector.hpp>
#include <muse_mcl/utility/delegate.hpp>

namespace muse_mcl {
/**
 * @brief The SampleInsertion class is used to fill up a particle set.
 *        The insertion object notifies usage and changes.
 */
template<typename StateT>
class SampleInsertion {
public:
    using notify_closed = delegate<void()>;
    using notify_update   = delegate<void(const Sample &)>;
    using sample_t        = Sample<StateT>;

    /**
     * @brief Insertion constructor.
     * @param data      - data structure to insert to
     * @param update    - on change notification callback
     * @param finshed   - on finish callback
     */
    SampleInsertion(std::buffered_vector<sample_t> &data,
              notify_update                         update,
              notify_closed                       close) :
        data_(data),
        open_(true),
        update_(update),
        close_(close)
    {
    }

    /**
     * @brief Insertion destructor.
     */
    virtual ~SampleInsertion()
    {
        if(open_) {
            close_();
        }
    }

    /**
     * @brief Insert by move.
     * @param sample - sample to be inserted
     */
    inline void insert(sample_t&& sample)
    {
        if(!open_)
            return;

        data_.emplace_back(std::move(sample));
        update_(sample);
    }

    /**
     * @brief Insert by reference.
     * @param sample - sample to be inserted
     */
    inline void insert(const sample_t &sample)
    {
        if(!open_)
            return;
        data_.push_back(sample);
        update_(sample);
    }

    /**
     * @brief Check wether insertion is still possible.
     * @return insertion is possible
     */
    inline bool canInsert() const
    {
        return data_.size() < data_.capacity() && open_;
    }

    /**
     * @brief Close an insertion. After that it cannot be used
     *        anymore to fill up a vector.
     */
    inline void close_()
    {
        if(open_) {
            open_ = false;
            close_();
        }
    }

    /**
     * @brief Return const reference to container the data is inserted to.
     * @return const reference to the data container
     */
    std::buffered_vector<sample_t> const & getData()
    {
        return data_;
    }

private:
    std::buffered_vector<sample_t> &data_;       /// the data container

    bool                            open_;       /// indicator if insertion is still open
    notify_update                   update_;     /// on update callback
    notify_closed                 close_;      /// on close / finish callback

};
}
#endif // SAMPLE_INSERTION_HPP
