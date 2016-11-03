#pragma once

#include <string>
#include <muse_amcl/plugins/plugin_manager.hpp>

namespace muse_amcl {
template<typename PluginType>
class PluginFactory {
public:
    PluginFactory() :
        plugin_manager(PluginType::Type()),
        nh_private_("~")
    {
        plugin_manager.load();
    }

    virtual typename PluginType::Ptr create(const std::string class_name)
    {
        auto constructor = plugin_manager.getConstructor(class_name);
        if(constructor) {
            return constructor();
        } else {
            std::cerr << "[PluginManager] :"
                      << " Cannot create '"  << class_name
                      << "' derivded from '" << PluginType::Type
                      << "'" << std::endl;
            return nullptr;
        }
    }

    static const std::string Type()
    {
        return PluginType::Type();
    }


protected:
    PluginManager<PluginType> plugin_manager;
    ros::NodeHandle           nh_private_;

};
}

