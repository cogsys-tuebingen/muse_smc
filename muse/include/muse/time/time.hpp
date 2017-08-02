#ifndef TIME_HPP
#define TIME_HPP

#include <chrono>

namespace muse {
class Time {
public:
    using clock_t = std::chrono::high_resolution_clock;
    using time_t  = clock_t::time_point;
    using duration_t = clock_t::duration;


    Time(const double seconds) :
        time_(std::chrono::nanoseconds(static_cast<int64_t>(seconds * 1e9)))
    {
    }

    Time(const time_t &time) :
        time_(time)
    {
    }

    inline time_t const & time() const
    {
        return time_;
    }

    inline double seconds() const
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(time_.time_since_epoch()).count() * 1e-9;
    }

    inline Time static now()
    {
        return Time(clock_t::now());
    }
private:
    time_t time_;


};
}
inline bool operator == (const muse::Time &a,
                         const muse::Time &b)
{
    return a.time() == b.time();
}

inline bool operator != (const muse::Time &a,
                         const muse::Time &b)
{
    return a.time() != b.time();
}

inline bool operator <= (const muse::Time &a,
                         const muse::Time &b)
{
    return a.time() <= b.time();
}

inline bool operator >= (const muse::Time &a,
                         const muse::Time &b)
{
    return a.time() >= b.time();
}

inline bool operator > (const muse::Time &a,
                         const muse::Time &b)
{
    return a.time() > b.time();
}

inline bool operator < (const muse::Time &a,
                        const muse::Time &b)
{
    return a.time() < b.time();
}
#endif // TIME_HPP
