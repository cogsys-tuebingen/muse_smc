#ifndef DATA_HPP
#define DATA_HPP

#include <memory>
#include <cslibs_time/time_frame.hpp>

namespace muse_smc {
class Data {
public:
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;

    Data(const std::string &_frame) :
        frame_(_frame)
    {
    }

    Data(const std::string              &frame,
         const cslibs_time::TimeFrame   &time_frame,
         const cslibs_time::Time        &time_received) :
        frame_(frame),
        time_frame_(time_frame),
        time_received_(time_received)
    {
    }

    virtual ~Data()
    {
    }

    inline const std::string & getFrame() const
    {
        return frame_;
    }

    inline const cslibs_time::TimeFrame & getTimeFrame() const
    {
        return time_frame_;
    }

    inline cslibs_time::Time const & getTimeReceived() const
    {
        return time_received_;
    }

    template<typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template<typename T>
    T const & as() const
    {
        return dynamic_cast<const T&>(*this);
    }

protected:
    Data()                  = delete;
    Data(const Data &other) = default;
    Data(Data &&other)      = default;

    std::string              frame_;
    cslibs_time::TimeFrame   time_frame_;
    cslibs_time::Time        time_received_;
};
}

#endif // DATA_HPP
