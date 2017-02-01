#ifndef TIME_FRAME_HPP
#define TIME_FRAME_HPP

#include <ros/time.h>

namespace muse_amcl {
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
    };
}

#endif // TIME_FRAME_HPP
