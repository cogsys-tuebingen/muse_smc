#pragma once

#include <muse_amcl/plugin/plugin_manager.hpp>
#include "update.hpp"

namespace muse_amcl {

class UpdateFunctionFactory
{
public:
    UpdateFunctionFactory();

    std::unique_ptr<Update> create(const std::string& name);

private:
    PluginManager<Update> plugin_manager_;
};

}
