#ifndef INSERTION_HPP
#define INSERTION_HPP

#include "particle.hpp"
#include <muse_amcl/utils/buffered_vector.hpp>
#include <muse_amcl/utils/delegate.hpp>

namespace muse_amcl {
class Insertion {
public:
    using notify_finished = delegate<void()>;
    using notify_update   = delegate<void(const Particle &)>;

    Insertion(std::buffered_vector<Particle> &data,
              notify_update                   update,
              notify_finished                 finshed) :
        data_(data),
        open_(true),
        update_(update),
        finished_(finshed)
    {
    }

    virtual ~Insertion()
    {
        if(open_) {
            finished_();
        }
    }

    inline void insert(Particle&& sample)
    {
        data_.emplace_back(std::move(sample));
        update_(sample);
    }

    inline void insert(const Particle &sample)
    {
        data_.push_back(sample);
        update_(sample);
    }

    inline bool canInsert() const
    {
        return data_.size() < data_.capacity();
    }

    inline void close()
    {
        if(open_) {
            finished_();
            open_ = false;
        } else {
            throw std::runtime_error("Insertion cannot be closed twice!");
        }
    }

    std::buffered_vector<Particle> const & set()
    {
        return data_;
    }

private:
    std::buffered_vector<Particle> &data_;

    bool                            open_;
    notify_update                   update_;
    notify_finished                 finished_;

};
}

#endif // INSERTION_HPP
