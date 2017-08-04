#ifndef SYNCHRONIZED_PRIORITY_QUEUE_HPP
#define SYNCHRONIZED_PRIORITY_QUEUE_HPP

#include <queue>
#include <mutex>

namespace muse_smc {
namespace synchronized {
template<typename _Tp, typename _Sequence = std::vector<_Tp>,
         typename _Compare  = std::less<typename _Sequence::value_type>>
class priority_queue
{
public:
    using mutex_t = std::mutex;
    using lock_t  = std::unique_lock<mutex_t>;

    priority_queue() = default;

    inline bool empty() const
    {
        lock_t l(mutex_);
        return q_.empty();
    }

    inline std::size_t size() const
    {
        lock_t l(mutex_);
        return q_.size();
    }

    inline _Tp pop() const
    {
        lock_t l(mutex_);
        _Tp t = q_.top();
        q_.pop();
        return t;
    }

    inline void emplace(const _Tp &t)
    {
        lock_t l(mutex_);
        q_.emplace(t);
    }

private:
    mutable mutex_t mutex_;
    std::priority_queue<_Tp, _Sequence, _Compare> q_;

};
}
}


#endif // SYNCHRONIZED_PRIORITY_QUEUE_HPP
