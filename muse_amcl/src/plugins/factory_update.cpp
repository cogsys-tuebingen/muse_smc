#include <muse_amcl/plugins/factory_update.h>
#include <muse_amcl/plugins/plugin_manager.hpp>

using namespace muse_amcl;

Update::Ptr UpdateFunctionFactory::create(const std::string& plugin_name,
                                          const std::string& class_name)
{
    Update::Ptr update = PluginFactory<Update>::create(class_name);
    if(update) {
        update->setup(plugin_name, nh_private_);
    }
    return update;
}
