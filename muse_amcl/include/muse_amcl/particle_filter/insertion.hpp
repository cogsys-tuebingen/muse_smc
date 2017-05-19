#ifndef INSERTION_HPP
#define INSERTION_HPP

#include "particle.hpp"
#include <muse_amcl/utils/buffered_vector.hpp>
#include <muse_amcl/utils/delegate.hpp>

namespace muse_mcl {
/**
 * @brief The Insertion class is used to fill up a particle set.
 *        The insertion object notifies usage and changes.
 */
class Insertion {
public:
    using notify_finished = delegate<void()>;
    using notify_update   = delegate<void(const Particle &)>;

    /**
     * @brief Insertion constructor.
     * @param data      - data structure to insert to
     * @param update    - on change notification callback
     * @param finshed   - on finish callback
     */
    Insertion(std::buffered_vector<Particle> &data,
              notify_update                   update,
              notify_finished                 finshed) :
        data_(data),
        open_(true),
        update_(update),
        closed_(finshed)
    {
    }

    /**
     * @brief Insertion destructor.
     */
    virtual ~Insertion()
    {
        if(open_) {
            closed_();
        }
    }

    /**
     * @brief Insert by move.
     * @param sample - sample to be inserted
     */
    inline void insert(Particle&& sample)
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
    inline void insert(const Particle &sample)
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
    inline void close()
    {
        if(open_) {
            open_ = false;
            closed_();
        } else {
            throw std::runtime_error("Insertion cannot be closed twice!");
        }
    }

    /**
     * @brief Return const reference to container the data is inserted to.
     * @return const reference to the data container
     */
    std::buffered_vector<Particle> const & getData()
    {
        return data_;
    }

private:
    std::buffered_vector<Particle> &data_;        /// the data container

    bool                            open_;        /// indicator if insertion is still open
    notify_update                   update_;      /// on update callback
    notify_finished                 closed_;      /// on close / finish callback

};
}

#endif // INSERTION_HPP
