#pragma once

#include <muse_amcl/plugins/plugin_manager.hpp>
#include <muse_amcl/plugins/propagation.hpp>

#include "factory.hpp"

namespace muse_amcl {

class PropagationFunctionFactory : public PluginFactory<Propagation>
{
public:
    Propagation::Ptr create(const std::string& plugin_name,
                            const std::string& class_name);

};
}
