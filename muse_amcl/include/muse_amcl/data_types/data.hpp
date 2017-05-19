#ifndef DATA_HPP
#define DATA_HPP

#include <memory>
#include <muse_amcl/data_types/time_frame.hpp>

namespace muse_mcl {
class Data {
public:
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;

    Data(const std::string &_frame) :
        frame_(_frame)
    {
    }

    Data(const std::string &frame,
         const TimeFrame   &time_frame) :
        frame_(frame),
        time_frame_(time_frame)
    {
    }

    virtual ~Data()
    {
    }

    inline const std::string & getFrame() const
    {
        return frame_;
    }

    inline const TimeFrame & getTimeFrame() const
    {
        return time_frame_;
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
    Data() = delete;
    Data(const Data &other) = default;
    Data(Data &&other) = default;

    std::string frame_;
    TimeFrame   time_frame_;
};
}

#endif /* DATA_HPP */
