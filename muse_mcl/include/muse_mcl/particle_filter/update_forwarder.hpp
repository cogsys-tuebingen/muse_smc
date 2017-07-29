#ifndef UPDATE_FORWARDER_HPP
#define UPDATE_FORWARDER_HPP

#include "update.hpp"
#include "particle_filter.hpp"

#include <muse_mcl/providers/provider_map.hpp>

namespace muse_mcl {
class UpdateForwarder {
public:
    using Ptr = std::shared_ptr<UpdateForwarder>;

    UpdateForwarder(const std::map<std::string, ModelUpdate::Ptr> &models,
                    const ParticleFilter::Ptr &filter) :
        models_(models),
        filter_(filter)
    {
    }

    inline void bind(const std::map<std::string, ProviderData::Ptr> &data_providers,
                     const std::map<std::string, ProviderMap::Ptr> &map_providers,
                     ros::NodeHandle &nh_private)
    {
        for(const auto &u : models_) {
            const std::string &model_name = u.first;
            const std::string data_provider_param    = model_name + "/data_provider";
            const std::string map_provider_param     = model_name + "/map_provider";

            const std::string data_provider_name     = nh_private.param<std::string>(data_provider_param, "");
            const std::string map_provider_name      = nh_private.param<std::string>(map_provider_param, "");

            ModelUpdate::Ptr  model = u.second;

            if(data_providers.find(data_provider_name) == data_providers.end())
                throw std::runtime_error("[UpdateForwarder]: Cannot find data provider '" + data_provider_name + "'!");

            if(map_providers.find(map_provider_name) == map_providers.end())
                throw std::runtime_error("[UpdateForwarder]: Cannot find map provider '" + map_provider_name + "'!");

            const ProviderData::Ptr &data_provider= data_providers.at(data_provider_name);
            const ProviderMap::Ptr  &map_provider = map_providers.at(map_provider_name);

            auto callback = [this, model, map_provider] (const Data::ConstPtr &data) {
                Map::ConstPtr map = map_provider->getMap();
                if(map) {
                    Update::Ptr u(new Update(data, map, model));
                    filter_->addUpdate(u);
                } else {
                    throw std::runtime_error("[UpdateForwarder]: Map was null!");
                }
            };

            connections_[model_name] = data_provider->connect(callback);
        }
    }

private:
    const std::map<std::string, ModelUpdate::Ptr> &models_;
    ParticleFilter::Ptr                            filter_;

    std::map<std::string, ProviderData::DataConnection::Ptr> connections_;
};
}


#endif // UPDATE_FORWARDER_HPP
