#ifndef DURATION_HPP
#define DURATION_HPP

#include <chrono>
#include <thread>

#include "ros/rate.h"

namespace muse {
class Duration {
public:
    using clock_t = std::chrono::high_resolution_clock;
    using time_t  = clock_t::time_point;
    using duration_t = clock_t::duration;

    Duration(const double seconds) :
        duration_(static_cast<int64_t>(std::floor(seconds * 1e9)))
    {
    }

    Duration(const int64_t nanoseconds) :
        duration_(nanoseconds)
    {
    }

    Duration(const duration_t duration) :
        duration_(duration)
    {
    }

    inline double seconds() const
    {
        return duration_.count() * 1e-9;

    }

    inline duration_t duration() const
    {
        return duration_;
    }

    inline int64_t nanoseconds() const
    {
        return duration_.count();
    }

    inline bool sleep() const
    {
        if(duration_ <= duration_t(0))
            return false;
        std::this_thread::sleep_for(duration_);
        return true;
    }

    inline muse::Duration operator + (const muse::Duration &other)
    {
        return muse::Duration(other.duration_ + duration_);
    }

    inline muse::Duration operator - (const muse::Duration &other)
    {
        return muse::Duration(other.duration_ - duration_);
    }

    inline muse::Duration operator * (const double s)
    {
        return muse::Duration(static_cast<int64_t>(std::floor(nanoseconds() * s)));
    }

    inline muse::Duration operator / (const double s)
    {
        return muse::Duration(static_cast<int64_t>(std::floor(nanoseconds() / s)));
    }

    inline bool operator == (const muse::Duration &other)
    {
        return duration_ == other.duration_;
    }

    inline bool operator != (const muse::Duration &other)
    {
        return duration_ != other.duration_;
    }

    inline bool operator <= (const muse::Duration &other)
    {
        return duration_ <= other.duration_;
    }

    inline bool operator >= (const muse::Duration &other)
    {
        return duration_ >= other.duration_;
    }

    inline bool operator > (const muse::Duration &other)
    {
        return duration_ > other.duration_;
    }

    inline bool operator < (const muse::Duration &other)
    {
        return duration_ < other.duration_;
    }

private:
    duration_t duration_;
};
}
#endif // DURATION_HPP
