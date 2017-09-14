#ifndef DURATION_HPP
#define DURATION_HPP

#include <chrono>
#include <thread>

#include "ros/rate.h"

namespace muse_smc {
class Duration {
public:
    using clock_t = std::chrono::high_resolution_clock;
    using time_t  = clock_t::time_point;
    using duration_t = clock_t::duration;

    Duration() :
        duration_(0l)
    {
    }

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
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count() * 1e-9;
    }

    inline double milliseconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count() * 1e-6;
    }

    inline int64_t nanoseconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration_).count();
    }

    inline duration_t duration() const
    {
        return duration_;
    }


    inline bool sleep() const
    {
        if(duration_ <= duration_t(0))
            return false;
        std::this_thread::sleep_for(duration_);
        return true;
    }

    inline bool isZero() const
    {
        return duration_.count() == 0l;
    }

    inline muse_smc::Duration operator + (const muse_smc::Duration &other)
    {
        return muse_smc::Duration(other.duration_ + duration_);
    }

    inline muse_smc::Duration operator - (const muse_smc::Duration &other)
    {
        return muse_smc::Duration(other.duration_ - duration_);
    }

    inline muse_smc::Duration operator * (const double s)
    {
        return muse_smc::Duration(static_cast<int64_t>(std::floor(nanoseconds() * s)));
    }

    inline muse_smc::Duration operator / (const double s)
    {
        return muse_smc::Duration(static_cast<int64_t>(std::floor(nanoseconds() / s)));
    }

    inline bool operator == (const muse_smc::Duration &other)
    {
        return duration_ == other.duration_;
    }

    inline bool operator != (const muse_smc::Duration &other)
    {
        return duration_ != other.duration_;
    }

    inline bool operator <= (const muse_smc::Duration &other)
    {
        return duration_ <= other.duration_;
    }

    inline bool operator >= (const muse_smc::Duration &other)
    {
        return duration_ >= other.duration_;
    }

    inline bool operator > (const muse_smc::Duration &other)
    {
        return duration_ > other.duration_;
    }

    inline bool operator < (const muse_smc::Duration &other)
    {
        return duration_ < other.duration_;
    }

private:
    duration_t duration_;
};
}
#endif // DURATION_HPP
