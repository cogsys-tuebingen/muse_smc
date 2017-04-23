#ifndef TIME_FRAME_HPP
#define TIME_FRAME_HPP

#include <ros/time.h>

namespace muse_amcl {
    struct TimeFrame {
        const ros::Time begin;
        const ros::Time end;

        TimeFrame() :
            begin(ros::Time::now()),
            end(begin)
        {
        }

        TimeFrame(const ros::Time &start,
                  const ros::Time &end) :
            begin(start),
            end(end)
        {
        }

        inline bool within(const ros::Time &time)
        {
            return time >= begin && time <= end;
        }

    };
}

#endif // TIME_FRAME_HPP
