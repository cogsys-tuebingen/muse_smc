#include <muse_amcl/plugins/factory_propagation.h>
#include <muse_amcl/plugins/plugin_manager.hpp>

using namespace muse_amcl;

Propagation::Ptr PropagationFunctionFactory::create(const std::string& plugin_name,
                                                    const std::string& class_name)
{
    Propagation::Ptr propagation = PluginFactory::create(class_name);
    if(propagation) {
        propagation->setup(plugin_name, nh_private_);
    }
    return propagation;
}
