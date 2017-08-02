#ifndef PLUGIN_FACTORY_HPP
#define PLUGIN_FACTORY_HPP

#include <string>

#include "plugin_manager.hpp"

namespace muse_mcl {
template<typename PluginType, typename ... setup_args_t>
class PluginFactory {
public:
    PluginFactory() :
        plugin_manager(PluginType::Type())
    {
        plugin_manager.load();
    }

    typename PluginType::Ptr create(const std::string &class_name,
                                    const std::string &plugin_name,
                                    const setup_args_t&...arguments)
    {
        auto constructor = plugin_manager.getConstructor(class_name);
        if(constructor) {
            typename PluginType::Ptr plugin = constructor();
            plugin->setup(plugin_name, arguments...);
            return plugin;
        } else {
            return nullptr;
        }
    }

    static const std::string Type()
    {
        return PluginType::Type();
    }

protected:
    PluginManager<PluginType> plugin_manager;

};
}
#endif /* PLUGIN_FACTORY_HPP */
