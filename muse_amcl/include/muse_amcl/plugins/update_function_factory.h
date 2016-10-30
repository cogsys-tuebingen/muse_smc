#pragma once

#include <muse_amcl/plugin/plugin_manager.hpp>
#include <muse_amcl/plugins/update.hpp>
#include "factory.hpp"

namespace muse_amcl {

class UpdateFunctionFactory : public PluginFactory<Update>
{
public:
    UpdateFunctionFactory();

    Update::Ptr create(const std::string& class_name);

};

}
