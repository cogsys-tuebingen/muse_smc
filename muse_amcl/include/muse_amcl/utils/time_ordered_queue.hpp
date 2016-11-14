#pragma once

#include <queue>
#include <memory>
#include <mutex>

namespace muse_amcl {
template<typename T>
class TimeOrderedQueue
{
public:
    struct Comparator {
        bool operator() (const T &lhs,
                         const T &rhs) const
        {
            /// lowest element should be "at the top"
            return lhs->stamp() > rhs->stamp();
        }
    };


    TimeOrderedQueue()          = default;
    virtual ~TimeOrderedQueue() = default;

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
