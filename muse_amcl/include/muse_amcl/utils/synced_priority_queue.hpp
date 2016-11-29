#ifndef SYNCED_PRIORITY_QUEUE_HPP
#define SYNCED_PRIORITY_QUEUE_HPP

#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>

namespace muse_amcl {
template<typename T, typename Comparator>
class SyncedPriorityQueue
{
public:
    SyncedPriorityQueue(const Comparator &comparator) :
        q_(comparator)
    {
    }

    SyncedPriorityQueue()
    {
    }

    virtual ~SyncedPriorityQueue()
    {

    }

    inline void push(const T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        q_.push(v);
        l.unlock();
        c_.notify_one();
    }

    inline void push(const T &&v)
    {
        std::unique_lock<std::mutex> l(m_);
        q_.push(std::move(v));
        l.unlock();
        c_.notify_one();
    }

    inline bool pop(T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        wait(l);
        if(q_.empty())
            return false;

        v = q_.top();
        q_.pop();
        return true;
    }

    inline bool top(T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        wait(l);
        if(q_.empty())
            return false;
        v = q_.top();
        return true;
    }

    inline std::size_t size() const
    {
        std::unique_lock<std::mutex> l(m_);
        return q_.size();
    }

    inline bool empty() const
    {
        std::unique_lock<std::mutex> l(m_);
        return q_.empty();
    }

private:
    inline void wait(std::unique_lock<std::mutex> &&lock)
    {
        while(q_.empty()) {
            c_.wait(lock);
        }
    }

    mutable std::mutex                                 m_;
    std::priority_queue<T, std::vector<T>, Comparator> q_;
    std::condition_variable                            c_;

};
}

#endif /* SYNCED_PRIORITY_QUEUE_HPP */
