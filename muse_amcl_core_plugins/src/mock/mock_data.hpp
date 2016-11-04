#pragma once

#include <string>
#include <muse_amcl/plugins/data.hpp>

namespace muse_amcl {

struct MockData : public Data
{
    std::string value;

};
}
