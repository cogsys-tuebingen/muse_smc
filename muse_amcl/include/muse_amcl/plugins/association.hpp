#pragma once

#include <map>
#include <string>
#include "../particle_filter/update.hpp"

namespace muse_amcl {
struct AssociationLoader {
    inline static void load(const std::map<std::string, Update::Ptr>  &update_functions,
                            const std::map<std::string, std::string>  &data_providers,
                            const std::map<std::string, std::string>  &map_providers)
    {
        for(const auto &u : update_functions) {
            const std::string &name = u.first;
            data_providers[name] = nh_private.param(name + "/" + "data_provider", "");
            map_providers[name]  = nh_private.param(name + "/" + "data_provider", "");
        }
    }
};
}
