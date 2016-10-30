#pragma once

#include <muse_amcl/plugin/plugin_manager.hpp>
#include <muse_amcl/plugins/propagation.hpp>

#include "factory.hpp"

namespace muse_amcl {

class PropagationFunctionFactory : public PluginFactory<Propagation>
{
public:
    PropagationFunctionFactory();

    Propagation::Ptr create(const std::string& class_name);

};

}
