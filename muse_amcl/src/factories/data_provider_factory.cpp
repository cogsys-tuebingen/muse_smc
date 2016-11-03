#include <muse_amcl/plugin_factories/data_provider_factory.h>

using namespace muse_amcl;

DataProvider::Ptr DataProviderFactory::create(const std::string& plugin_name,
                                              const std::string& class_name)
{
    DataProvider::Ptr provider = PluginFactory<DataProvider>::create(class_name);
    if(provider) {
        provider->setup(plugin_name, nh_private_);
    }
    return provider;
}
