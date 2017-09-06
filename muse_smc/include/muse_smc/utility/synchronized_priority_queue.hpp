#ifndef SYNCHRONIZED_PRIORITY_QUEUE_HPP
#define SYNCHRONIZED_PRIORITY_QUEUE_HPP

#include <queue>
#include <mutex>
#include <ext/pb_ds/priority_queue.hpp>

//using namespace std;

//int main(int argc, char *argv[]) {
//  __gnu_pbds::priority_queue<int, less<int>, __gnu_pbds::binary_heap_tag> pq;
//  cout << (typeid(__gnu_pbds::container_traits<decltype(pq)>::invalidation_guarantee) == typeid(__gnu_pbds::basic_invalidation_guarantee)) << endl;
//  // prints 1
//  cout << (typeid(__gnu_pbds::container_traits<__gnu_pbds::priority_queue<int, less<int>, __gnu_pbds::binary_heap_tag>>::invalidation_guarantee) == typeid(__gnu_pbds::basic_invalidation_guarantee)) << endl;
//  // prints 1
//  return 0;
//}

namespace muse_smc {
namespace synchronized {
template<typename _Tp, typename _Sequence = std::vector<_Tp>,
         typename _Compare  = std::less<typename _Sequence::value_type>>
class priority_queue
{
public:
    using mutex_t = std::mutex;
    using lock_t  = std::unique_lock<mutex_t>;
    using queue_t = std::priority_queue<_Tp, _Sequence, _Compare>;

    priority_queue() = default;

    inline bool empty() const
    {
        lock_t l(mutex_);
        return q_.empty();
    }

    inline bool hasElements() const
    {
        return !empty();
    }


    inline std::size_t size() const
    {
        lock_t l(mutex_);
        return q_.size();
    }

    inline _Tp pop()
    {
        lock_t l(mutex_);
        _Tp t = q_.top();
        q_.pop();
        return t;
    }

    inline _Tp const & top() const
    {
        lock_t(mutex_);
        return q_.top();
    }

    inline void emplace(const _Tp &t)
    {
        lock_t l(mutex_);
        q_.emplace(t);
    }

private:
    mutable mutex_t mutex_;
    queue_t q_;

};
}
}

#endif // SYNCHRONIZED_PRIORITY_QUEUE_HPP
