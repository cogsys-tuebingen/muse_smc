#pragma once

#include <memory>
#include <map>

#include "update.hpp"
#include "update_lambda.hpp"
#include "update_queue.hpp"

#include "../data_sources/data_provider.hpp"
#include "../data_sources/map_provider.hpp"


namespace muse_amcl {
class UpdateManager {
public:
    typedef std::shared_ptr<UpdateManager> Ptr;

    UpdateManager(const std::map<std::string, DataProvider::Ptr> &data_providers,
                  const std::map<std::string, MapProvider::Ptr>  &map_providers,
                  std::map<std::string, Update::Ptr>             &update_functions,
                  UpdateQueue &update_queue) :
        data_providers_(data_providers),
        map_providers_(map_providers),
        update_functions_(update_functions),
        update_queue_(update_queue)
    {
    }

    void bind(const std::map<std::string, std::string> &data_associations,
              const std::map<std::string, std::string> &map_associations)
    {
        for(auto &u : update_functions_) {
            const std::string &update_name = u.first;
            const std::string &data_provider_name = data_associations.at(update_name);
            const std::string &map_provider_name  = map_associations.at(update_name);

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

            Update::Ptr             &update_function = update_functions_.at(update_name);
            const DataProvider::Ptr &data_provider   = data_providers_.at(data_provider_name);
            const MapProvider::Ptr  &map_provider    = map_providers_.at(map_provider_name);

            auto callback = [this, update_function, map_provider] (const Data::ConstPtr &data) {
                auto f = [update_function, map_provider, data] (ParticleSet::WeightIterator set) {
                    return update_function->apply(data, map_provider->map(), set);
                };

                UpdateLambda u(f, data->stamp());
                update_queue_.push(u);
            };

            connections_[update_name] = data_provider->connect(callback);
        }
    }


private:
    /// TODO get rid of these map copies
    const std::map<std::string, DataProvider::Ptr> &data_providers_;
    const std::map<std::string, MapProvider::Ptr>  &map_providers_;
    std::map<std::string, Update::Ptr>             &update_functions_;
    UpdateQueue                                    &update_queue_;

    std::map<std::string, DataProvider::DataConnection::Ptr> connections_;

};
}
