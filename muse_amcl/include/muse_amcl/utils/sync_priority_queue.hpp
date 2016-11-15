#pragma once

#include <queue>
#include <memory>
#include <mutex>

namespace muse_amcl {
template<typename T, typename Comparator>
class SyncPriorityQueue
{
public:
    SyncPriorityQueue(const Comparator &comparator) :
        q_(comparator)
    {
    }

    virtual ~SyncPriorityQueue() = default;

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

private:
    std::mutex                                         m_;
    std::priority_queue<T, std::vector<T>, Comparator> q_;

};
}
