#ifndef SYNCED_PRIORITY_QUEUE_HPP
#define SYNCED_PRIORITY_QUEUE_HPP

#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace muse_mcl {
template<typename T, typename Comparator>
class SyncedPriorityQueue
{
public:
    using QueueType = std::priority_queue<T, std::vector<T>, Comparator>;

    SyncedPriorityQueue(const Comparator &comparator) :
        queue_(comparator),
        wait_(true)
    {
    }

    SyncedPriorityQueue() :
         wait_(true)
    {
    }

    virtual ~SyncedPriorityQueue()
    {
        wait_ = false;
        wait_condition_.notify_all();
    }

    inline void enableWaiting()
    {
        wait_ = true;
    }

    inline void disableWaiting()
    {
        wait_ = false;
        wait_condition_.notify_all();
    }

    inline void clear()
    {
        std::unique_lock<std::mutex> l(mutex_);
        queue_ = QueueType();
        l.unlock();
    }

    inline void push(const T &v)
    {
        std::unique_lock<std::mutex> l(mutex_);
        queue_.push(v);
        l.unlock();
        wait_condition_.notify_one();
    }

    inline void push(const T &&v)
    {
        std::unique_lock<std::mutex> l(mutex_);
        queue_.push(std::move(v));
        l.unlock();
        wait_condition_.notify_one();
    }

    inline bool pop(T &v)
    {
        std::unique_lock<std::mutex> l(mutex_);
        wait(l);
        if(queue_.empty())
            return false;

        v = queue_.top();
        queue_.pop();
        return true;
    }

    inline bool top(T &v)
    {
        std::unique_lock<std::mutex> l(mutex_);
        wait(l);
        if(queue_.empty())
            return false;
        v = queue_.top();
        return true;
    }

    inline std::size_t size() const
    {
        std::unique_lock<std::mutex> l(mutex_);
        return queue_.size();
    }

    inline bool empty() const
    {
        std::unique_lock<std::mutex> l(mutex_);
        return queue_.empty();
    }

private:
    inline void wait(std::unique_lock<std::mutex> &&lock)
    {
        while(queue_.empty()) {
            wait_condition_.wait(lock);

            if(!wait_)
                break;
        }
    }

    mutable std::mutex       mutex_;
    QueueType                queue_;
    std::condition_variable  wait_condition_;
    std::atomic_bool         wait_;

};
}

#endif /* SYNCED_PRIORITY_QUEUE_HPP */
