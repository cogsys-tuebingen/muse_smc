#pragma once

#include <muse_amcl/plugin/plugin_manager.hpp>
#include "propagation.hpp"

namespace muse_amcl {

class PropagationFunctionFactory
{
public:
    PropagationFunctionFactory();

    Propagation::Ptr create(const std::string& name);

private:
    PluginManager<Propagation> plugin_manager_;
};

}
