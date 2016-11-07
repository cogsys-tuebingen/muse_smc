#pragma once

#include "types/data_provider.hpp"
#include "factory.hpp"

namespace muse_amcl {
class DataProviderFactory : public PluginFactory<DataProvider>
{
public:
    DataProvider::Ptr create(const std::string &plugin_name,
                             const std::string &class_name);

};
}

