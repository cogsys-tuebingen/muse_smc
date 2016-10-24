#pragma once

#include <memory>

namespace muse_amcl {
struct Data {
    std::shared_ptr<Data> Ptr;
    std::shared_ptr<const Data> ConstPtr;

};
}
