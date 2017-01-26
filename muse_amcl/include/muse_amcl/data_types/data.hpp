#ifndef DATA_HPP
#define DATA_HPP

#include <memory>
#include <ros/time.h>

namespace muse_amcl {
class Data {
public:
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;

    Data(const std::string &_frame) :
        frame_(_frame),
        stamp_(ros::Time::now())
    {
    }

    Data(const std::string &_frame,
         const ros::Time   &_stamp) :
        frame_(_frame),
        stamp_(_stamp)
    {
    }

    virtual ~Data()
    {
    }

    inline const std::string & frame() const
    {
        return frame_;
    }

    inline const ros::Time & getStamp() const
    {
        return stamp_;
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
    Data(const Data &other) = delete;

    std::string frame_;
    ros::Time   stamp_;
};
}

#endif /* DATA_HPP */
