#pragma once

#include <muse_amcl/plugin/plugin_manager.hpp>
#include "update.hpp"

namespace muse_amcl {

class UpdateFunctionFactory
{
public:
    UpdateFunctionFactory();

    Update::Ptr create(const std::string& name);

private:
    PluginManager<Update> plugin_manager_;
};

}
