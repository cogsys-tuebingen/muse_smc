#ifndef TIME_FRAME_HPP
#define TIME_FRAME_HPP

#include <muse_smc/time/time.hpp>
#include <string>

namespace muse_smc {
struct TimeFrame {
    const Time start;
    const Time end;

    TimeFrame() :
        start(Time::now()),
        end(start)
    {
    }

    TimeFrame(const double start_seconds,
              const double end_seconds) :
        start(start_seconds),
        end(end_seconds)
    {
    }

    TimeFrame(const int64_t &start_nanoseconds,
              const int64_t &end_nanoseconds) :
        start(start_nanoseconds),
        end(end_nanoseconds)
    {
    }

    TimeFrame(const uint64_t &start_nanoseconds,
              const uint64_t &end_nanoseconds) :
        start(start_nanoseconds),
        end(end_nanoseconds)
    {
    }

    TimeFrame(const Time::time_t &start,
              const Time::time_t &end) :
        start(start),
        end(end)
    {
    }

    TimeFrame(const Time &start,
              const Time &end) :
        start(start),
        end(end)
    {
    }

    TimeFrame(const TimeFrame &other) :
        start(other.start),
        end(other.end)
    {
    }

    inline bool within(const Time &time) const
    {
        return time >= start && time <= end;
    }

    inline Duration duration() const
    {
        return end.time() - start.time();
    }
};
}

inline std::ostream & operator << (std::ostream &out, const muse_smc::TimeFrame &time)
{
    std::cout << "[" << time.start << "," << time.end << "]";
    return out;
}

#endif // TIME_FRAME_HPP
