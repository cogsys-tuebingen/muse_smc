#ifndef INSERTION_HPP
#define INSERTION_HPP

#include "particle.hpp"
#include "buffered_vector.hpp"

namespace muse_amcl {
template<typename Notifier>
class Insertion {
public:
    using notify_finished = void (Notifier::*)();
    using notify_update   = void (Notifier::*)(const Particle &);

    Insertion(std::buffered_vector<Particle> &data,
              Notifier                       &notifier,
              notify_update                   update,
              notify_finished                 finshed) :
        data_(data),
        notifier_(notifier),
        open_(true),
        update_(update),
        finished_(finshed)
    {
    }

    virtual ~Insertion()
    {
        if(open_) {
            (notifier_.*finished_)();
        }
    }

    inline void insert(const Particle &sample)
    {
        data_.emplace_back(sample);
        (notifier_.*update_)(sample);
    }

    inline bool canInsert() const
    {
        return data_.size() < data_.capacity();
    }

    inline void close()
    {
        if(open_) {
            (notifier_.*finished_)();
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
    Notifier                       &notifier_;

    bool                            open_;
    notify_update                   update_;
    notify_finished                 finished_;

};
}

#endif // INSERTION_HPP
