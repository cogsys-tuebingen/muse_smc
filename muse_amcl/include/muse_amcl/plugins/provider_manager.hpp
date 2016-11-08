#pragma once

#include <ros/node_handle.h>
#include <memory>
#include <boost/regex.hpp>

#include "factory.hpp"

namespace muse_amcl {
template<typename ProviderType>
class ProviderManager
{
public:
    typedef std::shared_ptr<ProviderManager> Ptr;

    struct LaunchEntry {
        std::string class_name;
        std::string base_class_name;
    };

    ProviderManager()
    {
    }

    virtual ~ProviderManager()
    {
    }

    bool load(ros::NodeHandle &nh_private)
    {
        std::string  name_space = nh_private.getNamespace();
        boost::regex class_regex("(" + ns + "/)(.*)(/class)");
        boost::regex base_class_regex("(" + ns + "/)(.*)(/base_class)");

        providers.clear();

        /// first parse the parameters
        std::vector<std::string>           params;
        std::map<std::string, LaunchEntry> plugins_found;
        nh_private.getParamNames(params);

        for(const std::string &p : params) {
            if(boost::regex_match(p.c_str(), match, class_regex)) {
                nh_private.getParam(p, plugins_found[match[2]].class_name);
            }
            if(boost::regex_match(p.c_str(), match, base_class_regex)) {
                nh_private.getParam(p, plugins_found[match[2]].base_class_name);
            }
        }

        /// all in the launch file entered plugins have been retrieved now
        /// now we load the ones related to this ProviderManager
        for(const auto &entry : plugins_found) {
            const std::string &name = entry.first;
            const std::string &base_class_name = e.second.base_class_name;
            const std::string &class_name = e.second.class_name;
            if(base_class_name == ProviderType::Type()) {
                providers[name] = factory.create(name, class_name);
            }
        }
    }

private:
    PluginFactory<ProviderType>                       factory;
    std::map<std::string, typename ProviderType::Ptr> providers;
};
}
