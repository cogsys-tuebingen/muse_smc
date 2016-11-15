#pragma once

#include <queue>
#include <memory>
#include <mutex>

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

    virtual ~SyncedPriorityQueue() = default;

    inline void push(const T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        q_.push(v);
    }

    inline T pop()
    {
        std::unique_lock<std::mutex> l(m_);
        T v = q_.top();
        q_.pop();
        return v;
    }

    inline void pop(T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        v = q_.top();
        q_.pop();
    }

    inline void top(T &v)
    {
        std::unique_lock<std::mutex> l(m_);
        v = q_.top();
    }

    inline T top()
    {
        std::unique_lock<std::mutex> l(m_);
        return q_.top();
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
    mutable std::mutex                                 m_;
    std::priority_queue<T, std::vector<T>, Comparator> q_;

};
}
