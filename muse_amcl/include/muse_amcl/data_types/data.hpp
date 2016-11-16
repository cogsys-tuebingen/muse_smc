#pragma once

#include <memory>
#include <chrono>

namespace muse_amcl {
class Data {
public:
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;

    Data(const std::string &_frame) :
        frame_(_frame),
        stamp_(std::chrono::system_clock::now())
    {
    }

    Data(const std::string &_frame,
         const std::chrono::time_point<std::chrono::system_clock> &_stamp) :
        frame_(_frame),
        stamp_(_stamp)
    {
    }

    virtual ~Data()
    {
    }

    inline std::string frame() const
    {
        return frame_;
    }

    inline std::chrono::time_point<std::chrono::system_clock> stamp() const
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
    std::chrono::time_point<std::chrono::system_clock> stamp_;
};
}
