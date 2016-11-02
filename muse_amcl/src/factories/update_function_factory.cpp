#include <muse_amcl/plugin_factories/update_function_factory.h>

using namespace muse_amcl;

UpdateFunctionFactory::UpdateFunctionFactory() :
    nh_private_("~")
{
}

Update::Ptr UpdateFunctionFactory::create(const std::string& plugin_name,
                                          const std::string& class_name)
{
    Update::Ptr update = PluginFactory<Update>::create(class_name);
    if(update) {
        update->setup(plugin_name, nh_private_);
    }
    return update;
}
