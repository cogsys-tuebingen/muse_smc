#ifndef STAMPED_HPP
#define STAMPED_HPP

#include <muse_smc/time/time.hpp>

namespace muse_smc {
template<typename T>
class Stamped
{
public:
    inline Stamped(const T &data,
                   const Time &time) :
        data_(data),
        time_(time)
    {
    }

    inline Time & stamp()
    {
        return time_;
    }

    inline Time const & stamp() const
    {
        return stamp;
    }

    inline T & data()
    {
        return data_;
    }

    inline T const & data() const
    {
        return data_;
    }

    inline operator T&()
    {
        return data_;
    }

    inline operator T*()
    {
        return &data_;
    }

    inline operator const T&() const
    {
        return data_;
    }

    inline operator T () const
    {
        return data_;
    }
private:
    T              data_;
    Time           time_;
};
}


#endif // STAMPED_HPP
