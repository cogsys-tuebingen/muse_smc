#ifndef PLUGIN_LOADER_HPP
#define PLUGIN_LOADER_HPP

#include "plugin_factory.hpp"
#include <map>
#include <ros/node_handle.h>
#include <boost/regex.hpp>

namespace muse_mcl {
class PluginLoader
{
public:
    PluginLoader(ros::NodeHandle &nh_private) :
        nh_private_(nh_private)
    {
        parseLaunchFile();
    }

    template<typename PluginType, typename ... Arguments>
    inline bool load(std::map<std::string, typename PluginType::Ptr> &plugins,
                     const Arguments&... arguments)
    {
        plugins.clear();

        /// all in the launch file entered plugins have been retrieved now
        /// now we load the ones related to this ProviderManager
        static PluginFactory<PluginType, Arguments...> factory;
        for(const auto &entry : plugins_found_) {
            const std::string &name = entry.first;
            const std::string &base_class_name = entry.second.base_class_name;
            const std::string &class_name = entry.second.class_name;
            if(base_class_name == PluginType::Type()) {
                plugins[name] = factory.create(class_name, name, arguments...);
            }
        }

        return plugins.size() > 0;
    }

    template<typename PluginType, typename ... Arguments>
    inline void load(typename PluginType::Ptr &plugin,
                     const Arguments&... arguments)
    {
        static PluginFactory<PluginType, Arguments...> factory;
        for(const auto &entry : plugins_found_) {
            const std::string &name = entry.first;
            const std::string &base_class_name = entry.second.base_class_name;
            const std::string &class_name = entry.second.class_name;
            if(base_class_name == PluginType::Type()) {
                plugin = factory.create(class_name, name, arguments...);
            }
        }
    }

    inline std::set<std::string> getFoundNames() const
    {
        std::set<std::string> found;
        for(const auto &entry : plugins_found_) {
            found.insert(entry.first);
        }
        return found;
    }

private:
    struct LaunchEntry {
        std::string class_name;
        std::string base_class_name;
    };

    ros::NodeHandle nh_private_;
    std::map<std::string, LaunchEntry> plugins_found_;

    inline void parseLaunchFile()
    {
        std::string  ns = nh_private_.getNamespace();
        boost::regex class_regex("(" + ns + "/)(.*)(/class)");
        boost::regex base_class_regex("(" + ns + "/)(.*)(/base_class)");

        /// first parse the parameters
        std::vector<std::string> params;
        nh_private_.getParamNames(params);

        boost::cmatch match;
        for(const std::string &p : params) {
            if(boost::regex_match(p.c_str(), match, class_regex)) {
                nh_private_.getParam(p, plugins_found_[match[2]].class_name);
            }
            if(boost::regex_match(p.c_str(), match, base_class_regex)) {
                nh_private_.getParam(p, plugins_found_[match[2]].base_class_name);
            }
        }
    }

};
}

#endif /* PLUGIN_LOADER_HPP */
