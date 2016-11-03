#pragma once

#include <memory>

namespace muse_amcl {
struct Data {
    typedef std::shared_ptr<Data> Ptr;
    typedef std::shared_ptr<const Data> ConstPtr;
};
}
