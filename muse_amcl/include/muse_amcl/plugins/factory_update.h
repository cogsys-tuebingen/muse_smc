#pragma once

#include "types/update.hpp"
#include "factory.hpp"

namespace muse_amcl {

class UpdateFunctionFactory : public PluginFactory<Update>
{
public:
    Update::Ptr create(const std::string& plugin_name,
                       const std::string& class_name);
};

}
