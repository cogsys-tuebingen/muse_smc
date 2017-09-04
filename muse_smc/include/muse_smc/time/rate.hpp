#ifndef RATE_HPP
#define RATE_HPP

#include <muse_smc/time/time.hpp>
#include <muse_smc/time/duration.hpp>

namespace muse_smc {
class Rate {
public:
    using clock_t = std::chrono::high_resolution_clock;
    using time_t  = clock_t::time_point;
    using duration_t = clock_t::duration;

    Rate() :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(std::numeric_limits<double>::infinity())
    {
    }

    Rate(const double rate) :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(1.0 / rate)
    {
    }

    Rate(const Duration &d) :
        start_(Time::now()),
        actual_cycle_time_(0.0),
        expected_cycle_time_(d)
    {
    }

    void reset()
    {
        start_ = Time::now();
    }

    Duration cycleTime() const
    {
        return actual_cycle_time_;
    }

    Duration expectedCycleTime() const
    {
        return expected_cycle_time_;
    }

    inline bool sleep()
    {
        Time expected_end = start_ + expected_cycle_time_;
        Time actual_end = Time::now();

        if (actual_end < start_)
        {
            expected_end = actual_end + expected_cycle_time_;
        }

        Duration sleep_time = expected_end - actual_end;
        actual_cycle_time_ = actual_end - start_;
        start_ = expected_end;

        if(sleep_time <= Duration(0.0))
        {
            if (actual_end > expected_end + expected_cycle_time_)
            {
                start_ = actual_end;
            }
            return false;
        }

        return sleep_time.sleep();
    }

private:
    Time     start_;
    Duration actual_cycle_time_;
    Duration expected_cycle_time_;
};
}


#endif // RATE_HPP
