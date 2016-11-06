#pragma once

#include <memory>

namespace muse_amcl {
struct Data {
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;

    virtual ~Data()
    {
    }

    template<typename T>
    bool isType() const
    {
        const T *t = dynamic_cast<const T*>(this);
        return t != nullptr;
    }

    template<typename T>
    T const * as() const
    {
        return dynamic_cast<const T*>(this);
    }

};
}
