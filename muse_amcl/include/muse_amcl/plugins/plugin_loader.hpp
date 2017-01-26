#ifndef PLUGIN_LOADER_HPP
#define PLUGIN_LOADER_HPP

#include "plugin_factory.hpp"
#include <map>
#include <ros/node_handle.h>
#include <boost/regex.hpp>

namespace muse_amcl {
template<typename PluginType, typename ... Arguments>
struct PluginLoader
{
    struct LaunchEntry {
        std::string class_name;
        std::string base_class_name;
    };

    inline static void parseLaunchFile(ros::NodeHandle &nh_private,
                                       std::map<std::string, LaunchEntry> &plugins_found)
    {
        std::string  ns = nh_private.getNamespace();
        boost::regex class_regex("(" + ns + "/)(.*)(/class)");
        boost::regex base_class_regex("(" + ns + "/)(.*)(/base_class)");

        /// first parse the parameters
        std::vector<std::string> params;
        nh_private.getParamNames(params);

        boost::cmatch match;
        for(const std::string &p : params) {
            if(boost::regex_match(p.c_str(), match, class_regex)) {
                nh_private.getParam(p, plugins_found[match[2]].class_name);
            }
            if(boost::regex_match(p.c_str(), match, base_class_regex)) {
                nh_private.getParam(p, plugins_found[match[2]].base_class_name);
            }
        }
    }

    inline static bool load(ros::NodeHandle &nh_private,
                            std::map<std::string, typename PluginType::Ptr> &plugins,
                            const Arguments&... arguments)
    {
        plugins.clear();

        std::map<std::string, LaunchEntry> plugins_found;
        parseLaunchFile(nh_private, plugins_found);

        /// all in the launch file entered plugins have been retrieved now
        /// now we load the ones related to this ProviderManager
        static PluginFactory<PluginType, Arguments...> factory; /// @TODO: make sure plugin manager stays alive!
        for(const auto &entry : plugins_found) {
            const std::string &name = entry.first;
            const std::string &base_class_name = entry.second.base_class_name;
            const std::string &class_name = entry.second.class_name;
            if(base_class_name == PluginType::Type()) {
                plugins[name] = factory.create(class_name, name, arguments...);
            }
        }

        return plugins.size() > 0;
    }

    static void load(ros::NodeHandle &nh_private,
                     typename PluginType::Ptr &plugin,
                     const Arguments&... arguments)
    {
        std::map<std::string, LaunchEntry> plugins_found;
        parseLaunchFile(nh_private, plugins_found);

        static PluginFactory<PluginType, Arguments...> factory;   /// @TODO: make sure plugin manager stays alive!
        for(const auto &entry : plugins_found) {
            const std::string &name = entry.first;
            const std::string &base_class_name = entry.second.base_class_name;
            const std::string &class_name = entry.second.class_name;
            if(base_class_name == PluginType::Type()) {
                plugin = factory.create(class_name, name, arguments...);
            }
        }
    }
};
}

#endif /* PLUGIN_LOADER_HPP */
