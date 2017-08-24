#ifndef PLUGIN_FACTORY_HPP
#define PLUGIN_FACTORY_HPP

#include <string>

#include "plugin_manager.hpp"

namespace muse_smc {
template<typename plugin_t>
class PluginID
{
public:
    static std::size_t getID()
    {
        static PluginID<plugin_t> id;
        return ++id.id_;
    }
private:
    PluginID() :
        id_(0)
    {
    }

    std::size_t id_;
};

template<typename plugin_t, typename ... setup_args_t>
class PluginFactory {
public:
    PluginFactory(const std::string &package_name) :
        plugin_manager(plugin_t::Type(), package_name),
        plugin_id_(0)
    {
        plugin_manager.load();
    }

    typename plugin_t::Ptr create(const std::string  &class_name,
                                  const std::string  &plugin_name,
                                  const setup_args_t &...arguments)
    {
        auto constructor = plugin_manager.getConstructor(class_name);
        if(constructor) {
            typename plugin_t::Ptr plugin = constructor();
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
        return plugin_t::Type();
    }

protected:
    PluginManager<plugin_t> plugin_manager;
    std::size_t             plugin_id_;
};

///// specialize without parameter pack

}
#endif /* PLUGIN_FACTORY_HPP */
