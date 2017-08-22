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

        inline std::string toString() const
        {
            return "[" + std::to_string(start.seconds()) + " -> " + std::to_string(end.seconds()) + "]";
        }

    };
}



#endif // TIME_FRAME_HPP
