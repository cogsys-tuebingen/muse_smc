#pragma once

#include <map>
#include <string>
#include <ros/node_handle.h>
#include "../particle_filter/update.hpp"
#include "../particle_filter/propagation.hpp"

namespace muse_amcl {
struct Associations {
    inline static void load(const Propagation::Ptr &propagation_function,
                            ros::NodeHandle        &nh_private,
                            std::string            &data_provider)
    {
            data_provider =
                    nh_private.param<std::string>(
                        propagation_function->name() + "/" + "data_provider", "");
    }

    inline static void load(const std::map<std::string, Update::Ptr>  &update_functions,
                            ros::NodeHandle &nh_private,
                            std::map<std::string, std::string>  &data_providers,
                            std::map<std::string, std::string>  &map_providers)
    {
        for(const auto &u : update_functions) {
            const std::string &name = u.first;
            data_providers[name] = nh_private.param<std::string>(name + "/" + "data_provider", "");
            map_providers[name]  = nh_private.param<std::string>(name + "/" + "map_provider", "");
        }
    }
};
}
