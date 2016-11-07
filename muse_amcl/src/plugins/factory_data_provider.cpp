#include <muse_amcl/plugins/factory_data_provider.h>
#include <muse_amcl/plugins/plugin_manager.hpp>

using namespace muse_amcl;

DataProvider::Ptr DataProviderFactory::create(const std::string& plugin_name,
                                              const std::string& class_name)
{
    DataProvider::Ptr provider = PluginFactory::create(class_name);
    if(provider) {
        provider->setup(plugin_name, nh_private_);
    }
    return provider;
}
