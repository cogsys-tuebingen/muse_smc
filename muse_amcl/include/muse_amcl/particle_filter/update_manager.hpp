#pragma once

#include <memory>
#include <map>

#include "update.hpp"
#include "../data_sources/data_provider.hpp"
#include "../data_sources/map_provider.hpp"
#include "update_lambda.hpp"


namespace muse_amcl {
class UpdateManager {
public:
    typedef std::shared_ptr<UpdateManager> Ptr;

    UpdateManager(const std::map<std::string, DataProvider::Ptr> &data_providers,
                  const std::map<std::string, MapProvider::Ptr>  &map_providers,
                  const std::map<std::string, Update::Ptr>       &update_functions) :
        data_providers_(data_providers),
        map_providers_(map_providers),
        update_functions_(update_functions)
    {
    }

    void bind(const std::map<std::string, std::string> &data_associations,
              const std::map<std::string, std::string> &map_associations)
    {
        for(auto &u : updates_functions_) {
            const std::string &update_name = u.first;
            const std::string &data_provider_name = data_associations[update_name];
            const std::string &map_provider_name  = map_associations[update_name];

            if(data_provider_name == "") {
                std::cerr << "[UpdateManager] Cannot bind update function '"
                          << update_name << "' due to missing data provider '"
                          << data_provider_name << "'!"
                          << std::endl;
                continue;
            }
            if(map_provider_name == "") {
                std::cerr << "[UpdateManager] Cannot bind update function '"
                          << update_name << "' due to missing map provider '"
                          << map_provider_name << "'!"
                          << std::endl;
                continue;
            }

            Update::Ptr       &update_function = update_functions_[update_name];
            DataProvider::Ptr &data_provider   = data_providers_[update_name];
            MapProvider::Ptr  &map_provider    = map_providers_[update_name];


            auto callback = [map_provider, update_function] (const Data::ConstPtr &data) {
                UpdateLambda u(std::bind(Update::apply, update_function, data, map_provider),
                               data->stamp());
                /// pushback in prio queue
            };

            connection_[update_name] = data_provider->connect(callback);
        }
    }


private:
    /// TODO get rid of these map copies
    std::map<std::string, DataProvider::Ptr>                 data_providers_;
    std::map<std::string, MapProvider::Ptr>                  map_providers_;
    std::map<std::string, Update::Ptr>                       update_functions_;
    std::map<std::string, DataProvider::DataConnection::Ptr> connections_;

};
}
