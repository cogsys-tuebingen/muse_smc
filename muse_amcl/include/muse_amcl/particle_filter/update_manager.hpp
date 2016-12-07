#ifndef UPDATE_MANAGER_HPP
#define UPDATE_MANAGER_HPP

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

    UpdateManager(std::map<std::string, Update::Ptr> &update_functions,
                  UpdateQueue &update_queue) :
        update_functions_(update_functions),
        update_queue_(update_queue)
    {
    }

    void bind(const std::map<std::string, DataProvider::Ptr> &data_providers,
              const std::map<std::string, MapProvider::Ptr>  &map_providers,
              ros::NodeHandle  &nh_private)
    {
        for(auto &u : update_functions_) {
            const std::string &update_name           = u.first;
            const std::string data_provider_param    = update_name + "/data_provider";
            const std::string map_provider_param     = update_name + "/map_provider";

            const std::string data_provider_name     = nh_private.param<std::string>(data_provider_param, "");
            const std::string map_provider_name      = nh_private.param<std::string>(map_provider_param, "");

            Update::Ptr             &update_function = update_functions_.at(update_name);
            const DataProvider::Ptr &data_provider   = data_providers.at(data_provider_name);
            const MapProvider::Ptr  &map_provider    = map_providers.at(map_provider_name);

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
    std::map<std::string, Update::Ptr>                      &update_functions_;
    UpdateQueue                                             &update_queue_;

    std::map<std::string, DataProvider::DataConnection::Ptr> connections_;

};
}

#endif /* UPDATE_MANAGER_HPP */
