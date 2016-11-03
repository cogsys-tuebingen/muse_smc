#pragma once

#include <muse_amcl/plugins/plugin_manager.hpp>
#include <muse_amcl/plugins/update.hpp>
#include "factory.hpp"

namespace muse_amcl {

class UpdateFunctionFactory : public PluginFactory<Update>
{
public:
    Update::Ptr create(const std::string& plugin_name,
                       const std::string& class_name);
};

}
