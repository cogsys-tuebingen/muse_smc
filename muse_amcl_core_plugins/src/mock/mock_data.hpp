#pragma once

#include <string>
#include <muse_amcl/plugins/types/data.hpp>

namespace muse_amcl {

struct MockData : public Data
{
    typedef std::shared_ptr<MockData const> ConstPtr;

    std::string value;

};
}
