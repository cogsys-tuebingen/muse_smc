#ifndef TIME_FRAME_HPP
#define TIME_FRAME_HPP

#include <ros/time.h>
#include <string>

namespace muse_mcl {
    struct TimeFrame {
        const ros::Time start;
        const ros::Time end;

        TimeFrame() :
            start(ros::Time::now()),
            end(start)
        {
        }

        TimeFrame(const ros::Time &start,
                  const ros::Time &end) :
            start(start),
            end(end)
        {
        }

        TimeFrame(const TimeFrame &other) :
            start(other.start),
            end(other.end)
        {
        }

        inline bool within(const ros::Time &time) const
        {
            return time >= start && time <= end;
        }

        inline ros::Duration duration() const
        {
            return end - start;
        }

        inline std::string toString() const
        {
            return "[" + std::to_string(start.toSec()) + " -> " + std::to_string(end.toSec()) + "]";
        }

    };
}



#endif // TIME_FRAME_HPP
