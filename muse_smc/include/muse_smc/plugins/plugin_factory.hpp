#ifndef PLUGIN_FACTORY_HPP
#define PLUGIN_FACTORY_HPP

#include <string>

#include "plugin_manager.hpp"

namespace muse_smc {
template<typename PluginType, typename ... setup_args_t>
class PluginFactory {
public:
    PluginFactory() :
        plugin_manager(PluginType::Type()),
        plugin_id_(0)
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
            plugin->setName(plugin_name);
            plugin->setId(++plugin_id_);
            plugin->setup(arguments...);
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
    std::size_t               plugin_id_;
};
}
#endif /* PLUGIN_FACTORY_HPP */
